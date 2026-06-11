package org.firstinspires.ftc.teamcode.core.runtime

import kotlin.math.max

/**
 * The named phases of [Robot.loop], in execution order. [label] is the
 * telemetry/WPILOG name stem (`loop/<label>Nanos`, `<label> ms`).
 */
enum class LoopPhase(val label: String) {
    CLEAR_CACHES("clearCaches"),
    PERIODIC("periodic"),
    INPUT("input"),
    CONTROL("control"),
    SCHEDULER("scheduler"),
    WRITE_HARDWARE("writeHardware"),
    TELEMETRY("telemetry"),
    RECORD("record"),
}

/**
 * Per-tick breakdown of where [Robot.loop] spends its time. One instance is
 * owned by [Robot] and overwritten in place every tick — no per-loop
 * allocation. Latest durations and rolling maxima are nanoseconds, indexed
 * by [LoopPhase].
 *
 * The phase durations sum to the time spent *inside* [Robot.loop]; the
 * remainder up to [totalNanos] is [overheadNanos] — loop dispatch and
 * whatever the FTC event loop steals between ticks.
 *
 * This exists to answer "which phase owns the loop time?" with data instead
 * of guesswork. Surface it via telemetry while diagnosing loop speed; ignore
 * it once the loop is healthy. Latest values show the most recent tick;
 * maxima retain spikes until telemetry publishes them ([resetMaxima]).
 */
class LoopProfile {

    private val latest = LongArray(LoopPhase.entries.size)
    private val maxima = LongArray(LoopPhase.entries.size)

    /** The most recent tick's duration for [phase]. */
    operator fun get(phase: LoopPhase): Long = latest[phase.ordinal]

    /** The peak duration for [phase] since the last [resetMaxima]. */
    fun max(phase: LoopPhase): Long = maxima[phase.ordinal]

    internal operator fun set(phase: LoopPhase, nanos: Long) {
        latest[phase.ordinal] = nanos
        maxima[phase.ordinal] = max(maxima[phase.ordinal], nanos)
    }

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
        get() = (totalNanos - latest.sum()).coerceAtLeast(0)

    fun resetMaxima() {
        maxima.fill(0)
        maxTotalNanos = 0
        maxOverheadNanos = 0
    }
}
