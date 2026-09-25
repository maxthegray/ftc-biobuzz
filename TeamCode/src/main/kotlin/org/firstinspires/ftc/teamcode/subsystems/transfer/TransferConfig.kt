package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.bylazar.configurables.annotations.Configurable

/**
 * Middle-motor powers and blocker servo settings, persisted under `transfer`.
 * Placeholders until the transfer is built: tune in Panels.
 */
@Configurable
object TransferConfig {

    private const val DEFAULT_STAGE_POWER = 0.4
    private const val DEFAULT_FEED_POWER = 1.0
    private const val DEFAULT_REVERSE_POWER = -0.6
    private const val DEFAULT_BLOCKER_CLOSED_POSITION = 0.0
    private const val DEFAULT_BLOCKER_OPEN_POSITION = 0.5
    private const val DEFAULT_BLOCKER_TRAVEL_MS = 150.0

    /** Pushes balls up against the closed blocker. */
    @JvmField var stagePower: Double = DEFAULT_STAGE_POWER

    /** Pushes balls past the open blocker into the turret. */
    @JvmField var feedPower: Double = DEFAULT_FEED_POWER

    @JvmField var reversePower: Double = DEFAULT_REVERSE_POWER

    @JvmField var blockerClosedPosition: Double = DEFAULT_BLOCKER_CLOSED_POSITION
    @JvmField var blockerOpenPosition: Double = DEFAULT_BLOCKER_OPEN_POSITION

    /** Time for the blocker to swing open; a feed runs no motors before it has passed. */
    @JvmField var blockerTravelMs: Double = DEFAULT_BLOCKER_TRAVEL_MS

    fun resetDefaults() {
        stagePower = DEFAULT_STAGE_POWER
        feedPower = DEFAULT_FEED_POWER
        reversePower = DEFAULT_REVERSE_POWER
        blockerClosedPosition = DEFAULT_BLOCKER_CLOSED_POSITION
        blockerOpenPosition = DEFAULT_BLOCKER_OPEN_POSITION
        blockerTravelMs = DEFAULT_BLOCKER_TRAVEL_MS
    }

    internal val safeStagePower: Double get() = power(stagePower, DEFAULT_STAGE_POWER)
    internal val safeFeedPower: Double get() = power(feedPower, DEFAULT_FEED_POWER)
    internal val safeReversePower: Double get() = power(reversePower, DEFAULT_REVERSE_POWER)
    internal val safeBlockerClosedPosition: Double get() = position(blockerClosedPosition, DEFAULT_BLOCKER_CLOSED_POSITION)
    internal val safeBlockerOpenPosition: Double get() = position(blockerOpenPosition, DEFAULT_BLOCKER_OPEN_POSITION)
    internal val safeBlockerTravelMs: Double
        get() = if (blockerTravelMs.isFinite() && blockerTravelMs >= 0.0) blockerTravelMs else DEFAULT_BLOCKER_TRAVEL_MS

    private fun power(value: Double, default: Double) = if (value.isFinite()) value.coerceIn(-1.0, 1.0) else default
    private fun position(value: Double, default: Double) = if (value.isFinite()) value.coerceIn(0.0, 1.0) else default
}
