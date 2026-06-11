package org.firstinspires.ftc.teamcode.core.estimation

import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Estimator-specific behavior: axis weights and the during-follow blend
 * policy. The shared latency-compensation/gating math is covered through
 * the LocalizerSubsystem facade tests.
 */
class PoseEstimatorTest {

    private val clock = FakeClock(start = 0L)
    private var currentPose = Pose2d(0.0, 0.0, 0.0)
    private var following = false
    private val events = mutableListOf<String>()
    private val estimator = PoseEstimator(
        currentPose = { currentPose },
        applyPose = { currentPose = it },
        clock = clock,
        onEvent = events::add,
        isFollowing = { following },
    )

    private fun sample(timestampNanos: Long, pose: Pose2d) {
        clock.now = timestampNanos
        currentPose = pose
        estimator.sample(timestampNanos, pose)
    }

    private fun apply(
        measured: Pose2d,
        translationWeight: Double = 1.0,
        headingWeight: Double = 1.0,
        maxJumpInches: Double = Double.MAX_VALUE,
        maxJumpRadians: Double = Double.MAX_VALUE,
    ): CorrectionResult = estimator.applyCorrection(
        measured = measured,
        timestampNanos = clock.now,
        blend = 1.0,
        maxJumpInches = maxJumpInches,
        maxJumpRadians = maxJumpRadians,
        translationWeight = translationWeight,
        headingWeight = headingWeight,
    )

    @Test
    fun headingOnlyCorrectionLeavesTranslationAlone() {
        sample(0L, Pose2d(10.0, 20.0, 0.0))

        val result = apply(
            Pose2d(50.0, 60.0, Math.toRadians(45.0)),
            translationWeight = 0.0,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertEquals(10.0, currentPose.x, 1e-9)
        assertEquals(20.0, currentPose.y, 1e-9)
        assertEquals(Math.toRadians(45.0), currentPose.heading, 1e-9)
    }

    @Test
    fun translationOnlyCorrectionLeavesHeadingAlone() {
        sample(0L, Pose2d(0.0, 0.0, 1.0))

        val result = apply(
            Pose2d(8.0, -3.0, 2.5),
            headingWeight = 0.0,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertEquals(8.0, currentPose.x, 1e-9)
        assertEquals(-3.0, currentPose.y, 1e-9)
        assertEquals(1.0, currentPose.heading, 1e-9)
    }

    @Test
    fun zeroWeightAxisIsNotGated() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))

        // Translation is wildly off, but the measurement doesn't claim to
        // know translation — it must not be rejected for it.
        val result = apply(
            Pose2d(500.0, 500.0, Math.toRadians(5.0)),
            translationWeight = 0.0,
            maxJumpInches = 12.0,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertEquals(Math.toRadians(5.0), currentPose.heading, 1e-9)
        assertEquals(0.0, currentPose.x, 1e-9)
    }

    @Test
    fun weightedAxisStillGates() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))

        val result = apply(
            Pose2d(500.0, 0.0, 0.0),
            maxJumpInches = 12.0,
        )

        assertEquals(CorrectionResult.REJECTED_JUMP, result)
        assertEquals(0.0, currentPose.x, 1e-9)
    }

    @Test
    fun partialWeightsScaleTheBlendPerAxis() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))

        val result = apply(
            Pose2d(10.0, 0.0, Math.toRadians(20.0)),
            translationWeight = 0.5,
            headingWeight = 0.25,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertEquals(5.0, currentPose.x, 1e-9)
        assertEquals(Math.toRadians(5.0), currentPose.heading, 1e-9)
    }

    @Test
    fun followingScalesAcceptedCorrectionsDown() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        following = true

        // followingBlendScale default is 0.25 → a blend-1.0 correction of
        // 8 inches applies 2 inches while a path is running.
        val result = apply(Pose2d(8.0, 0.0, 0.0))

        assertEquals(CorrectionResult.APPLIED, result)
        assertEquals(2.0, currentPose.x, 1e-9)
        assertTrue(events.any { "following" in it })
    }

    @Test
    fun notFollowingAppliesFullBlend() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        following = false

        apply(Pose2d(8.0, 0.0, 0.0))

        assertEquals(8.0, currentPose.x, 1e-9)
    }
}
