package org.firstinspires.ftc.teamcode.core.runtime

/**
 * Per-tick breakdown of where [Robot.loop] spends its time. One instance is
 * owned by [Robot] and overwritten in place every tick — no per-loop
 * allocation. All durations are nanoseconds, for the most recent tick only.
 *
 * The five phase fields sum to the time spent *inside* [Robot.loop]; the
 * remainder up to [totalNanos] is [overheadNanos] — gamepad polling and the
 * telemetry flush, which run in [OpModeBase] outside [Robot.loop].
 *
 * This exists to answer "which phase owns the loop time?" with data instead
 * of guesswork. Surface it via telemetry while diagnosing loop speed; ignore
 * it once the loop is healthy. Note that with throttled telemetry these are
 * *sampled* values — fine for spotting which phase dominates, less so for
 * catching rare spikes.
 */
class LoopProfile {
    /** Lynx bulk-cache clear at the top of the tick. */
    var clearCachesNanos: Long = 0
        internal set

    /** All subsystems' `periodic()` — pure reads / state updates. */
    var periodicNanos: Long = 0
        internal set

    /** The op-mode's `onLoop()` — gamepad handling, command scheduling, telemetry buffering. */
    var controlNanos: Long = 0
        internal set

    /** Ivy `Scheduler.execute()` — one tick of every running command. */
    var schedulerNanos: Long = 0
        internal set

    /** All subsystems' `writeHardware()` — includes Pedro's `Follower.update()`. */
    var writeHardwareNanos: Long = 0
        internal set

    /** Full loop wall-clock, matching [Robot.lastLoopNanos]. */
    var totalNanos: Long = 0
        internal set

    /** Time outside [Robot.loop]: gamepad polling + telemetry flush in [OpModeBase]. */
    val overheadNanos: Long
        get() = (totalNanos - clearCachesNanos - periodicNanos - controlNanos -
            schedulerNanos - writeHardwareNanos).coerceAtLeast(0)
}
