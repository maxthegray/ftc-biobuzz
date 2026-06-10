package org.firstinspires.ftc.teamcode.core.subsystems.localization

/**
 * Minimal Pinpoint read/write surface used by [SRSHubPinpointLocalizer].
 *
 * Keeping the localizer behind this interface lets host-side tests script
 * Pinpoint readings without constructing the SRSHub driver.
 */
interface PinpointSource {
    val xMm: Float
    val yMm: Float
    val headingRad: Float
    val xVelMmPerSec: Float
    val yVelMmPerSec: Float
    val headingVelRadPerSec: Float

    fun setPose(xMm: Float, yMm: Float, headingRad: Float)
    fun resetImu()
}
