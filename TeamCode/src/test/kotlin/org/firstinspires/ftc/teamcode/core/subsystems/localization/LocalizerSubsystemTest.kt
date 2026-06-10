package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.subsystems.drive.fakeFollower
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class LocalizerSubsystemTest {

    private val clock = FakeClock(start = 0L)
    private val follower = fakeFollower()
    private val localizer = LocalizerSubsystem(follower, clock)

    @Test
    fun correctionPreservesStraightLineMotionSinceMeasurement() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(200_000_000L, Pose(5.0, 0.0, 0.0))

        val applied = localizer.applyCorrection(Pose(100.0, 10.0, 0.0), 0L)

        assertTrue(applied)
        assertPose(Pose(105.0, 10.0, 0.0), follower.pose)
    }

    @Test
    fun correctionPreservesPureRotationSinceMeasurement() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(0.0, 0.0, PI / 2.0))

        val applied = localizer.applyCorrection(Pose(20.0, 30.0, Math.toRadians(10.0)), 0L)

        assertTrue(applied)
        assertPose(Pose(20.0, 30.0, Math.toRadians(100.0)), follower.pose)
    }

    @Test
    fun correctionComposesTranslationInMeasuredFrame() {
        sample(0L, Pose(10.0, 0.0, PI / 2.0))
        sample(100_000_000L, Pose(10.0, 5.0, PI))

        val applied = localizer.applyCorrection(Pose(100.0, 100.0, 0.0), 0L)

        assertTrue(applied)
        assertPose(Pose(105.0, 100.0, PI / 2.0), follower.pose)
    }

    @Test
    fun correctionRejectsStaleTimestamp() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(1_000_000_000L, Pose(10.0, 0.0, 0.0))

        val applied = localizer.applyCorrection(
            measured = Pose(100.0, 0.0, 0.0),
            timestampNanos = 0L,
            maxAgeNanos = 500_000_000L,
        )

        assertFalse(applied)
        assertPose(Pose(10.0, 0.0, 0.0), follower.pose)
    }

    @Test
    fun correctionUsesInterpolatedHistoricalPose() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(10.0, 0.0, 0.0))
        sample(200_000_000L, Pose(20.0, 0.0, 0.0))

        val applied = localizer.applyCorrection(Pose(100.0, 0.0, 0.0), 50_000_000L)

        assertTrue(applied)
        assertPose(Pose(115.0, 0.0, 0.0), follower.pose)
    }

    private fun sample(timestampNanos: Long, pose: Pose) {
        clock.now = timestampNanos
        follower.setPose(pose)
        localizer.periodic()
    }

    private fun assertPose(expected: Pose, actual: Pose) {
        assertEquals(expected.x, actual.x, EPS)
        assertEquals(expected.y, actual.y, EPS)
        assertEquals(expected.heading, actual.heading, EPS)
    }

    private companion object {
        const val EPS = 1e-6
    }
}
