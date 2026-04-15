package org.firstinspires.ftc.teamcode.general.core

import com.pedropathing.ivy.Scheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.general.hardware.BulkReadManager

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
        lastTickEndNs = System.nanoTime()
        loopCount = 0
    }

    /**
     * Single main-loop tick. Order is fixed and tuned for correctness:
     *  1. Clear Lynx bulk caches so all reads this tick return fresh data
     *  2. [SubsystemBase.periodic] — pure reads, state updates
     *  3. [Scheduler.execute] — commands tick, deciding motor/servo targets
     *  4. [SubsystemBase.writeHardware] — flush those targets to hardware
     *
     * Returns the tick duration in nanoseconds so callers can surface it
     * via telemetry.
     */
    fun loop(): Long {
        bulkRead.clearCaches()
        for (s in subsystems) s.periodic()
        Scheduler.execute()
        for (s in subsystems) s.writeHardware()

        val now = System.nanoTime()
        lastLoopNanos = now - lastTickEndNs
        lastTickEndNs = now
        loopCount++
        return lastLoopNanos
    }

    /** Shutdown everything. Exceptions are swallowed so all subsystems get a chance to clean up. */
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
