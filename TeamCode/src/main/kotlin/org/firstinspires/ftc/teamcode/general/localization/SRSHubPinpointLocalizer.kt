package org.firstinspires.ftc.teamcode.general.localization

import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.general.hardware.SRSHubSubsystem
import kotlin.math.hypot

private const val MM_PER_INCH = 25.4f

/**
 * Pedro [Localizer] backed by a GoBilda Pinpoint plugged into an SRSHub.
 *
 * The SRSHub does the actual I2C transaction once per tick (in
 * [SRSHubSubsystem.periodic]). This adapter just translates the cached
 * pose/velocity into Pedro's coordinate convention (inches + radians) so
 * the follower can use it as a drop-in localizer in `Constants.kt`.
 *
 * Heading sign and pod offsets are baked into the Pinpoint at registration
 * time; this class does not invert anything.
 */
class SRSHubPinpointLocalizer(
    private val pinpoint: SRSHubSubsystem.PinpointHandle,
) : Localizer {

    private var startPose: Pose = Pose()
    private var totalHeading: Double = 0.0
    private var lastRawHeading: Double = 0.0

    override fun getPose(): Pose = Pose(
        startPose.x + pinpoint.xMm / MM_PER_INCH,
        startPose.y + pinpoint.yMm / MM_PER_INCH,
        startPose.heading + pinpoint.headingRad.toDouble(),
    )

    override fun getVelocity(): Pose = Pose(
        pinpoint.xVelMmPerSec / MM_PER_INCH.toDouble(),
        pinpoint.yVelMmPerSec / MM_PER_INCH.toDouble(),
        pinpoint.headingVelRadPerSec.toDouble(),
    )

    override fun getVelocityVector(): Vector {
        val vxIn = pinpoint.xVelMmPerSec / MM_PER_INCH.toDouble()
        val vyIn = pinpoint.yVelMmPerSec / MM_PER_INCH.toDouble()
        return Vector(hypot(vxIn, vyIn), Math.atan2(vyIn, vxIn))
    }

    override fun setStartPose(setStart: Pose) {
        startPose = setStart
    }

    override fun setPose(setPose: Pose) {
        pinpoint.setPose(
            ((setPose.x - startPose.x) * MM_PER_INCH).toFloat(),
            ((setPose.y - startPose.y) * MM_PER_INCH).toFloat(),
            (setPose.heading - startPose.heading).toFloat(),
        )
    }

    override fun update() {
        // SRSHubSubsystem.periodic() drives the I2C tick. We only need to keep
        // totalHeading wrapped across the ±π boundary for getTotalHeading().
        val raw = pinpoint.headingRad.toDouble()
        var delta = raw - lastRawHeading
        if (delta > Math.PI) delta -= 2 * Math.PI
        if (delta < -Math.PI) delta += 2 * Math.PI
        totalHeading += delta
        lastRawHeading = raw
    }

    override fun getTotalHeading(): Double = totalHeading

    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0

    override fun resetIMU() {
        pinpoint.resetImu()
    }

    override fun getIMUHeading(): Double = pinpoint.headingRad.toDouble()

    override fun isNAN(): Boolean =
        pinpoint.xMm.isNaN() || pinpoint.yMm.isNaN() || pinpoint.headingRad.isNaN()
}
