package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Scheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.util.Alliance

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
class Robot(val hardwareMap: HardwareMap) {

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

    private var initialized = false

    /**
     * Register a subsystem. Must happen before [init] (i.e. in the op-mode's
     * `configure()`) — a subsystem registered later would silently never get
     * its `init`, so this throws instead. Order matters only for
     * tie-breaking in telemetry output.
     */
    fun <T : SubsystemBase> register(subsystem: T): T {
        check(!initialized) {
            "Cannot register ${subsystem.name} after Robot.init() — register subsystems in configure()."
        }
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
        initialized = true
    }

    /**
     * Called once at the moment the OpMode actually starts running (after
     * init but before the first loop tick). Resets the loop timer so the
     * first [lastLoopNanos] reading isn't skewed by init time.
     */
    fun start() {
        lastTickEndNs = System.nanoTime()
        loopCount = 0
    }

    /**
     * Single main-loop tick. Order is fixed and tuned for correctness:
     *  1. Clear Lynx bulk caches so all reads this tick return fresh data
     *  2. [SubsystemBase.periodic] — pure reads, state updates
     *  3. [control] — OpMode code reacts to fresh state and schedules commands
     *  4. [Scheduler.execute] — commands tick, deciding motor/servo targets
     *  5. [SubsystemBase.writeHardware] — flush those targets to hardware
     *
     * Returns the tick duration in nanoseconds so callers can surface it
     * via telemetry.
     */
    fun loop(control: () -> Unit = {}): Long {
        var phaseStart = System.nanoTime()

        bulkRead.clearCaches()
        phaseStart = mark(phaseStart) { profile.clearCachesNanos = it }

        for (s in subsystems) s.periodic()
        phaseStart = mark(phaseStart) { profile.periodicNanos = it }

        control()
        phaseStart = mark(phaseStart) { profile.controlNanos = it }

        Scheduler.execute()
        phaseStart = mark(phaseStart) { profile.schedulerNanos = it }

        for (s in subsystems) s.writeHardware()
        val now = System.nanoTime()
        profile.writeHardwareNanos = now - phaseStart

        lastLoopNanos = now - lastTickEndNs
        profile.totalNanos = lastLoopNanos
        lastTickEndNs = now
        loopCount++
        return lastLoopNanos
    }

    /** Record the duration of the just-finished phase and return the new phase start. */
    private inline fun mark(start: Long, assign: (Long) -> Unit): Long {
        val now = System.nanoTime()
        assign(now - start)
        return now
    }

    /**
     * Shutdown everything. Exceptions are swallowed so all subsystems get a
     * chance to clean up.
     *
     * Note: Ivy's [Scheduler.reset] clears its queues *without* calling
     * `end()` on running commands (Ivy 1.0.0 has no public cancel-all), so
     * command-level cleanup does not run here. [SubsystemBase.stop] is the
     * safety net — every subsystem must leave its hardware safe regardless
     * of what command was mid-flight.
     */
    fun stop() {
        Scheduler.reset()
        for (s in subsystems) {
            try { s.stop() } catch (_: Throwable) { /* best-effort */ }
        }
    }

    /** Average loop frequency in Hz over the most recent tick. */
    val loopHz: Double
        get() = if (lastLoopNanos > 0) 1e9 / lastLoopNanos else 0.0
}
