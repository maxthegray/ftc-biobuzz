package org.firstinspires.ftc.teamcode.core.runtime

import kotlin.math.max

/**
 * Per-tick breakdown of where [Robot.loop] spends its time. One instance is
 * owned by [Robot] and overwritten in place every tick — no per-loop
 * allocation. Latest durations and rolling maxima are nanoseconds.
 *
 * The seven phase fields sum to the time spent *inside* [Robot.loop]; the
 * remainder up to [totalNanos] is [overheadNanos] — loop dispatch and
 * whatever the FTC event loop steals between ticks.
 *
 * This exists to answer "which phase owns the loop time?" with data instead
 * of guesswork. Surface it via telemetry while diagnosing loop speed; ignore
 * it once the loop is healthy. The latest fields show the most recent tick;
 * max fields retain spikes until telemetry publishes them.
 */
class LoopProfile {
    /** Lynx bulk-cache clear at the top of the tick. */
    var clearCachesNanos: Long = 0
        internal set(value) {
            field = value
            maxClearCachesNanos = max(maxClearCachesNanos, value)
        }

    var maxClearCachesNanos: Long = 0
        private set

    /** All subsystems' `periodic()` — pure reads / state updates. */
    var periodicNanos: Long = 0
        internal set(value) {
            field = value
            maxPeriodicNanos = max(maxPeriodicNanos, value)
        }

    var maxPeriodicNanos: Long = 0
        private set

    /** Gamepad edge detection + trigger polling (commands scheduled by bindings included). */
    var inputNanos: Long = 0
        internal set(value) {
            field = value
            maxInputNanos = max(maxInputNanos, value)
        }

    var maxInputNanos: Long = 0
        private set

    /** The op-mode's `onLoop()` — command scheduling, telemetry buffering. */
    var controlNanos: Long = 0
        internal set(value) {
            field = value
            maxControlNanos = max(maxControlNanos, value)
        }

    var maxControlNanos: Long = 0
        private set

    /** Ivy `Scheduler.execute()` — one tick of every running command. */
    var schedulerNanos: Long = 0
        internal set(value) {
            field = value
            maxSchedulerNanos = max(maxSchedulerNanos, value)
        }

    var maxSchedulerNanos: Long = 0
        private set

    /** All subsystems' `writeHardware()` — includes Pedro's `Follower.update()`. */
    var writeHardwareNanos: Long = 0
        internal set(value) {
            field = value
            maxWriteHardwareNanos = max(maxWriteHardwareNanos, value)
        }

    var maxWriteHardwareNanos: Long = 0
        private set

    /** Telemetry publish + (throttled) transmit at the bottom of the tick. */
    var telemetryNanos: Long = 0
        internal set(value) {
            field = value
            maxTelemetryNanos = max(maxTelemetryNanos, value)
        }

    var maxTelemetryNanos: Long = 0
        private set

    /** Full loop wall-clock, matching [Robot.lastLoopNanos]. */
    var totalNanos: Long = 0
        internal set(value) {
            field = value
            maxTotalNanos = max(maxTotalNanos, value)
            maxOverheadNanos = max(maxOverheadNanos, overheadNanos)
        }

    var maxTotalNanos: Long = 0
        private set

    var maxOverheadNanos: Long = 0
        private set

    /** Residue of [totalNanos] not covered by a named phase. */
    val overheadNanos: Long
        get() = (totalNanos - clearCachesNanos - periodicNanos - inputNanos -
            controlNanos - schedulerNanos - writeHardwareNanos - telemetryNanos)
            .coerceAtLeast(0)

    fun resetMaxima() {
        maxClearCachesNanos = 0
        maxPeriodicNanos = 0
        maxInputNanos = 0
        maxControlNanos = 0
        maxSchedulerNanos = 0
        maxWriteHardwareNanos = 0
        maxTelemetryNanos = 0
        maxTotalNanos = 0
        maxOverheadNanos = 0
    }
}
