package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Scheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Robot is the single source of truth for the robot's hardware + subsystems
 * + command scheduler for the lifetime of one OpMode.
 *
 * Responsibilities:
 *  - Own the list of [SubsystemBase]s
 *  - Put every [LynxModule] into MANUAL bulk read mode and clear the cache
 *    exactly once per main-loop tick
 *  - Drive Ivy's [Scheduler] each tick
 *  - Provide a consistent lifecycle: init → start → loop (read/periodic/tick/write) → stop
 *
 * [OpModeBase] handles the actual OpMode plumbing; you rarely construct
 * Robot yourself.
 */
class Robot(
    val hardwareMap: HardwareMap,
    private val clock: Clock = Clock.SYSTEM,
) {

    private val subsystems = mutableListOf<SubsystemBase>()
    val bulkRead = BulkReadManager(hardwareMap)

    var alliance: Alliance = Alliance.RED

    /** Monotonic tick counter, useful for log throttling. */
    var loopCount: Long = 0
        private set

    /** Wall-clock duration of the most recent loop in nanoseconds. */
    var lastLoopNanos: Long = 0
        private set

    /** Per-phase breakdown of the most recent loop. Overwritten in place each tick. */
    val profile = LoopProfile()

    private var lastTickEndNs: Long = 0

    /** Register a subsystem. Order matters only for tie-breaking in telemetry output. */
    fun <T : SubsystemBase> register(subsystem: T): T {
        subsystems += subsystem
        return subsystem
    }

    fun subsystems(): List<SubsystemBase> = subsystems

    /**
     * Initialise every registered subsystem. Any exception from a subsystem's
     * init is rethrown after logging so the OpMode fails loudly rather than
     * silently running with half-initialised hardware.
     */
    fun init() {
        bulkRead.init()
        Scheduler.reset()
        for (s in subsystems) s.init(hardwareMap)
    }

    /**
     * Called once at the moment the OpMode actually starts running (after
     * init but before the first loop tick). Resets the loop timer so the
     * first [lastLoopNanos] reading isn't skewed by init time.
     */
    fun start() {
        lastTickEndNs = clock.nanos()
        loopCount = 0
    }

    /**
     * Single main-loop tick. Order is fixed and tuned for correctness:
     *  1. Clear Lynx bulk caches so all reads this tick return fresh data
     *  2. [SubsystemBase.periodic] — pure reads, state updates
     *  3. [input] — gamepad edge detection + trigger polling, so bindings
     *     (including sensor-backed triggers) react to *this* tick's data
     *  4. [control] — OpMode code reacts to fresh state and schedules commands
     *  5. [Scheduler.execute] — commands tick, deciding motor/servo targets
     *  6. [SubsystemBase.writeHardware] — flush those targets to hardware
     *  7. [telemetry] — publish + (throttled) transmit, measured like any
     *     other phase instead of hiding in loop overhead
     *
     * Note: because [telemetry] runs before this tick's total is computed,
     * any loop-total it publishes is the *previous* tick's. Per-phase numbers
     * are current.
     *
     * Returns the tick duration in nanoseconds.
     */
    fun loop(
        input: () -> Unit = {},
        control: () -> Unit = {},
        telemetry: () -> Unit = {},
    ): Long {
        var phaseStart = clock.nanos()

        bulkRead.clearCaches()
        phaseStart = mark(phaseStart) { profile.clearCachesNanos = it }

        for (s in subsystems) s.periodic()
        phaseStart = mark(phaseStart) { profile.periodicNanos = it }

        input()
        phaseStart = mark(phaseStart) { profile.inputNanos = it }

        control()
        phaseStart = mark(phaseStart) { profile.controlNanos = it }

        Scheduler.execute()
        phaseStart = mark(phaseStart) { profile.schedulerNanos = it }

        for (s in subsystems) s.writeHardware()
        phaseStart = mark(phaseStart) { profile.writeHardwareNanos = it }

        telemetry()
        val now = clock.nanos()
        profile.telemetryNanos = now - phaseStart

        lastLoopNanos = now - lastTickEndNs
        profile.totalNanos = lastLoopNanos
        lastTickEndNs = now
        loopCount++
        return lastLoopNanos
    }

    /**
     * One init-phase tick: fresh bulk read + subsystem reads, nothing else.
     * No commands run and nothing is written to hardware — the robot must
     * not move before start. Gamepad edge updates happen in [OpModeBase]'s
     * init loop with trigger polling disabled, so bindings wired in
     * `configure()` cannot start commands early.
     */
    fun initTick() {
        bulkRead.clearCaches()
        for (s in subsystems) s.periodic()
    }

    /** Record the duration of the just-finished phase and return the new phase start. */
    private inline fun mark(start: Long, assign: (Long) -> Unit): Long {
        val now = clock.nanos()
        assign(now - start)
        return now
    }

    /**
     * Shutdown everything. Ivy's `Scheduler.reset()` clears command state without
     * calling command end handlers, so critical hardware cleanup belongs in each
     * subsystem's [SubsystemBase.stop]. Exceptions are swallowed so all subsystems
     * get a chance to clean up.
     */
    fun stop() {
        if (loopCount > 0) {
            for (s in subsystems) {
                try { s.persistState() } catch (_: Throwable) { /* best-effort */ }
            }
        }
        Scheduler.reset()
        for (s in subsystems) {
            try { s.stop() } catch (_: Throwable) { /* best-effort */ }
        }
    }

    /** Average loop frequency in Hz over the most recent tick. */
    val loopHz: Double
        get() = if (lastLoopNanos > 0) 1e9 / lastLoopNanos else 0.0
}
