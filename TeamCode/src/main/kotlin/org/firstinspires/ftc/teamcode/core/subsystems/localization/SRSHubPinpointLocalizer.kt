package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.core.hardware.SRSHubSubsystem
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

private const val MM_PER_INCH = 25.4

/**
 * Pedro [Localizer] backed by a GoBilda Pinpoint plugged into an SRSHub.
 *
 * The SRSHub does the actual I2C transaction once per tick (in
 * [SRSHubSubsystem.periodic]). This adapter just translates the cached
 * pose/velocity into Pedro's coordinate convention (inches + radians) so
 * the follower can use it as a drop-in localizer in `Constants.kt`.
 *
 * The Pinpoint accumulates displacement in its own reset frame; everything
 * it reports is re-rooted at [startPose] by rotating that displacement by
 * `startPose.heading` — the same convention as Pedro's reference
 * `PinpointLocalizer`. Heading sign and pod offsets are baked into the
 * Pinpoint at registration time; this class does not invert anything.
 */
class SRSHubPinpointLocalizer(
    private val pinpoint: SRSHubSubsystem.PinpointHandle,
) : Localizer {

    private var startPose: Pose = Pose()
    private var totalHeading: Double = 0.0
    private var lastRawHeading: Double = 0.0

    override fun getPose(): Pose {
        val h = startPose.heading
        val dx = pinpoint.xMm / MM_PER_INCH
        val dy = pinpoint.yMm / MM_PER_INCH
        return Pose(
            startPose.x + dx * cos(h) - dy * sin(h),
            startPose.y + dx * sin(h) + dy * cos(h),
            startPose.heading + pinpoint.headingRad.toDouble(),
        )
    }

    override fun getVelocity(): Pose {
        val h = startPose.heading
        val vx = pinpoint.xVelMmPerSec / MM_PER_INCH
        val vy = pinpoint.yVelMmPerSec / MM_PER_INCH
        return Pose(
            vx * cos(h) - vy * sin(h),
            vx * sin(h) + vy * cos(h),
            pinpoint.headingVelRadPerSec.toDouble(),
        )
    }

    override fun getVelocityVector(): Vector {
        val vx = pinpoint.xVelMmPerSec / MM_PER_INCH
        val vy = pinpoint.yVelMmPerSec / MM_PER_INCH
        return Vector(hypot(vx, vy), atan2(vy, vx) + startPose.heading)
    }

    override fun setStartPose(setStart: Pose) {
        startPose = setStart
    }

    override fun setPose(setPose: Pose) {
        // Rotate the field-frame offset from startPose by -startPose.heading
        // back into the Pinpoint's own frame before writing it to the device.
        val h = startPose.heading
        val dx = setPose.x - startPose.x
        val dy = setPose.y - startPose.y
        pinpoint.setPose(
            ((dx * cos(h) + dy * sin(h)) * MM_PER_INCH).toFloat(),
            ((-dx * sin(h) + dy * cos(h)) * MM_PER_INCH).toFloat(),
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
