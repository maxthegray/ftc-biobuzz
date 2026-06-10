package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class SRSHubPinpointLocalizerTest {

    private class FakePinpointSource : PinpointSource {
        override var xMm: Float = 0f
        override var yMm: Float = 0f
        override var headingRad: Float = 0f
        override var xVelMmPerSec: Float = 0f
        override var yVelMmPerSec: Float = 0f
        override var headingVelRadPerSec: Float = 0f
        var resetCount = 0

        override fun setPose(xMm: Float, yMm: Float, headingRad: Float) {
            this.xMm = xMm
            this.yMm = yMm
            this.headingRad = headingRad
        }

        override fun resetImu() {
            resetCount++
        }
    }

    private val source = FakePinpointSource()
    private val localizer = SRSHubPinpointLocalizer(source)

    @Test
    fun identityAtZeroStart() {
        source.xMm = 25.4f
        source.yMm = -50.8f
        source.headingRad = 0.5f

        val pose = localizer.getPose()

        assertEquals(1.0, pose.x, EPS)
        assertEquals(-2.0, pose.y, EPS)
        assertEquals(0.5, pose.heading, EPS)
    }

    @Test
    fun deviceTranslationRotatesByStartHeading() {
        localizer.setStartPose(Pose(10.0, 20.0, PI / 2.0))
        source.xMm = 25.4f

        val pose = localizer.getPose()

        assertEquals(10.0, pose.x, EPS)
        assertEquals(21.0, pose.y, EPS)
        assertEquals(PI / 2.0, pose.heading, EPS)
    }

    @Test
    fun setPoseRoundTripsThroughDeviceFrame() {
        val target = Pose(12.0, 2.0, 2.1)
        localizer.setStartPose(Pose(5.0, -3.0, 0.7))

        localizer.setPose(target)
        val pose = localizer.getPose()

        assertEquals(target.x, pose.x, EPS)
        assertEquals(target.y, pose.y, EPS)
        assertEquals(target.heading, pose.heading, EPS)
    }

    @Test
    fun velocityRotatesByStartHeading() {
        localizer.setStartPose(Pose(0.0, 0.0, PI / 2.0))
        source.xVelMmPerSec = 25.4f
        source.headingVelRadPerSec = -0.2f

        val velocity = localizer.getVelocity()
        val vector = localizer.getVelocityVector()

        assertEquals(0.0, velocity.x, EPS)
        assertEquals(1.0, velocity.y, EPS)
        assertEquals(-0.2, velocity.heading, EPS)
        assertEquals(1.0, vector.magnitude, EPS)
        assertEquals(PI / 2.0, vector.theta, EPS)
    }

    @Test
    fun nanReadingsAreReported() {
        assertFalse(localizer.isNAN)
        source.xMm = Float.NaN
        assertTrue(localizer.isNAN)
    }

    @Test
    fun totalHeadingUnwrapsAcrossPiBoundary() {
        source.headingRad = 3.10f
        localizer.update()
        source.headingRad = -3.10f
        localizer.update()

        assertEquals(3.1831853, localizer.totalHeading, 1e-5)
    }

    @Test
    fun resetImuDelegatesToSource() {
        localizer.resetIMU()
        assertEquals(1, source.resetCount)
    }

    private companion object {
        const val EPS = 1e-6
    }
}
