package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import kotlin.math.hypot
import org.firstinspires.ftc.teamcode.core.util.rotateTranslation

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
    private val pinpoint: PinpointSource,
) : Localizer {

    private var startPose: Pose = Pose()
    private var totalHeading: Double = 0.0
    private var lastRawHeading: Double = 0.0

    override fun getPose(): Pose {
        val (x, y) = deviceToWorld(
            pinpoint.xMm / MM_PER_INCH.toDouble(),
            pinpoint.yMm / MM_PER_INCH.toDouble(),
        )
        return Pose(x, y, MathFunctions.normalizeAngle(startPose.heading + rawHeading()))
    }

    override fun getVelocity(): Pose {
        val (vx, vy) = rotateByStart(
            pinpoint.xVelMmPerSec / MM_PER_INCH.toDouble(),
            pinpoint.yVelMmPerSec / MM_PER_INCH.toDouble(),
        )
        return Pose(vx, vy, pinpoint.headingVelRadPerSec.toDouble())
    }

    override fun getVelocityVector(): Vector {
        val velocity = getVelocity()
        return Vector(
            hypot(velocity.x, velocity.y),
            MathFunctions.normalizeAngle(Math.atan2(velocity.y, velocity.x)),
        )
    }

    override fun setStartPose(setStart: Pose) {
        startPose = Pose(setStart.x, setStart.y, MathFunctions.normalizeAngle(setStart.heading))
        lastRawHeading = rawHeading()
        totalHeading = lastRawHeading
    }

    override fun setPose(setPose: Pose) {
        val dx = setPose.x - startPose.x
        val dy = setPose.y - startPose.y
        val (deviceX, deviceY) = rotateTranslation(dx, dy, -startPose.heading)
        val deviceHeading = MathFunctions.normalizeAngle(setPose.heading - startPose.heading)
        pinpoint.setPose(
            (deviceX * MM_PER_INCH).toFloat(),
            (deviceY * MM_PER_INCH).toFloat(),
            deviceHeading.toFloat(),
        )
        lastRawHeading = deviceHeading
        totalHeading = deviceHeading
    }

    override fun update() {
        // SRSHubSubsystem.periodic() drives the I2C tick. We only need to keep
        // totalHeading wrapped across the ±π boundary for getTotalHeading().
        val raw = rawHeading()
        var delta = raw - lastRawHeading
        if (delta > Math.PI) delta -= 2 * Math.PI
        if (delta < -Math.PI) delta += 2 * Math.PI
        totalHeading += delta
        lastRawHeading = raw
    }

    override fun getTotalHeading(): Double = startPose.heading + totalHeading

    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0

    override fun resetIMU() {
        pinpoint.resetImu()
    }

    override fun getIMUHeading(): Double = rawHeading()

    override fun isNAN(): Boolean =
        pinpoint.xMm.isNaN() || pinpoint.yMm.isNaN() || pinpoint.headingRad.isNaN()

    private fun rawHeading(): Double = MathFunctions.normalizeAngle(pinpoint.headingRad.toDouble())

    private fun deviceToWorld(deviceX: Double, deviceY: Double): Pair<Double, Double> {
        val (rotX, rotY) = rotateByStart(deviceX, deviceY)
        return Pair(startPose.x + rotX, startPose.y + rotY)
    }

    private fun rotateByStart(x: Double, y: Double): Pair<Double, Double> {
        return rotateTranslation(x, y, startPose.heading)
    }
}
