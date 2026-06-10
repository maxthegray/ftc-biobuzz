package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.subsystems.drive.fakeFollower
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem.CorrectionResult
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class LocalizerSubsystemTest {

    private val clock = FakeClock(start = 0L)
    private val follower = fakeFollower()
    private val events = mutableListOf<String>()
    private val localizer = LocalizerSubsystem(follower, clock, onEvent = events::add)

    @Test
    fun correctionPreservesStraightLineMotionSinceMeasurement() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(200_000_000L, Pose(5.0, 0.0, 0.0))

        val result = applyUngated(Pose(100.0, 10.0, 0.0), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose(Pose(105.0, 10.0, 0.0), follower.pose)
    }

    @Test
    fun correctionPreservesPureRotationSinceMeasurement() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(0.0, 0.0, PI / 2.0))

        val result = applyUngated(Pose(20.0, 30.0, Math.toRadians(10.0)), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose(Pose(20.0, 30.0, Math.toRadians(100.0)), follower.pose)
    }

    @Test
    fun correctionComposesTranslationInMeasuredFrame() {
        sample(0L, Pose(10.0, 0.0, PI / 2.0))
        sample(100_000_000L, Pose(10.0, 5.0, PI))

        val result = applyUngated(Pose(100.0, 100.0, 0.0), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose(Pose(105.0, 100.0, PI / 2.0), follower.pose)
    }

    @Test
    fun correctionRejectsStaleTimestamp() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(1_000_000_000L, Pose(10.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose(100.0, 0.0, 0.0),
            timestampNanos = 0L,
            maxAgeNanos = 500_000_000L,
        )

        assertEquals(CorrectionResult.STALE, result)
        assertPose(Pose(10.0, 0.0, 0.0), follower.pose)
        assertTrue(events.any { "stale" in it })
    }

    @Test
    fun correctionReportsMissingHistory() {
        // No samples recorded at all.
        val result = localizer.applyCorrection(Pose(1.0, 1.0, 0.0), 0L)

        assertEquals(CorrectionResult.NO_HISTORY, result)
    }

    @Test
    fun correctionUsesInterpolatedHistoricalPose() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(10.0, 0.0, 0.0))
        sample(200_000_000L, Pose(20.0, 0.0, 0.0))

        val result = applyUngated(Pose(100.0, 0.0, 0.0), 50_000_000L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose(Pose(115.0, 0.0, 0.0), follower.pose)
    }

    @Test
    fun correctionRejectsJumpBeyondPositionGate() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(5.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose(100.0, 0.0, 0.0),
            timestampNanos = 0L,
            blend = 1.0,
            maxJumpInches = 12.0,
            maxJumpRadians = Double.MAX_VALUE,
        )

        assertEquals(CorrectionResult.REJECTED_JUMP, result)
        assertPose(Pose(5.0, 0.0, 0.0), follower.pose)
        assertTrue(events.any { "jump" in it })
    }

    @Test
    fun correctionRejectsJumpBeyondHeadingGate() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(0.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose(0.0, 0.0, Math.toRadians(90.0)),
            timestampNanos = 0L,
            blend = 1.0,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Math.toRadians(30.0),
        )

        assertEquals(CorrectionResult.REJECTED_JUMP, result)
        assertPose(Pose(0.0, 0.0, 0.0), follower.pose)
    }

    @Test
    fun blendAppliesFractionOfCorrection() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(0.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose(10.0, 4.0, Math.toRadians(20.0)),
            timestampNanos = 0L,
            blend = 0.5,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Double.MAX_VALUE,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose(Pose(5.0, 2.0, Math.toRadians(10.0)), follower.pose)
    }

    @Test
    fun repeatedBlendedCorrectionsConvergeOnMeasurement() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        repeat(8) { i ->
            val t = (i + 1) * 100_000_000L
            sample(t, follower.pose)
            localizer.applyCorrection(
                measured = Pose(10.0, 0.0, 0.0),
                timestampNanos = t,
                blend = 0.5,
                maxJumpInches = Double.MAX_VALUE,
                maxJumpRadians = Double.MAX_VALUE,
            )
        }
        assertTrue("expected convergence, got ${follower.pose.x}", follower.pose.x > 9.5)
    }

    private fun applyUngated(measured: Pose, timestampNanos: Long): CorrectionResult =
        localizer.applyCorrection(
            measured = measured,
            timestampNanos = timestampNanos,
            blend = 1.0,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Double.MAX_VALUE,
        )

    /** Pose history is sampled in writeHardware, right after Follower.update(). */
    private fun sample(timestampNanos: Long, pose: Pose) {
        clock.now = timestampNanos
        follower.setPose(pose)
        localizer.writeHardware()
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
