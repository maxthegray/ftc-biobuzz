package org.firstinspires.ftc.teamcode.core.util

/**
 * Monotonic match timer with injectable clock for host-side tests.
 */
class MatchTimer(private val clock: Clock = Clock.SYSTEM) {
    private var startNanos: Long? = null

    fun start() {
        startNanos = clock.nanos()
    }

    val elapsedSec: Double
        get() = startNanos?.let { (clock.nanos() - it) / 1e9 } ?: 0.0

    fun remainingSec(matchLengthSec: Double): Double =
        (matchLengthSec - elapsedSec).coerceAtLeast(0.0)

    fun inEndgame(thresholdSec: Double = 30.0, matchLengthSec: Double = 120.0): Boolean =
        elapsedSec >= matchLengthSec - thresholdSec
}
