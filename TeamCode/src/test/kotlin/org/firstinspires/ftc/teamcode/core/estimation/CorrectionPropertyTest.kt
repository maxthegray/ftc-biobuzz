package org.firstinspires.ftc.teamcode.core.estimation

import kotlin.math.abs
import kotlin.random.Random
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngleSigned
import org.firstinspires.ftc.teamcode.core.geometry.shortestAngleDelta
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/** Randomized invariants for the correction pipeline. */
class CorrectionPropertyTest {

    private fun randomPose(random: Random): Pose2d = Pose2d(
        random.nextDouble(0.0, 141.5),
        random.nextDouble(0.0, 141.5),
        random.nextDouble(-2.0 * Math.PI, 2.0 * Math.PI),
    )

    private class Rig {
        val clock = FakeClock(start = 0L)
        var pose = Pose2d.ZERO
        val estimator = PoseEstimator(
            currentPose = { pose },
            applyPose = { pose = it },
            clock = clock,
        )

        fun seed(p: Pose2d) {
            pose = p
            estimator.sample(clock.now, p)
        }
    }

    @Test
    fun gateDecisionIsMonotonicInTheLimit() {
        val random = Random(99)
        repeat(300) {
            val current = randomPose(random)
            val measured = randomPose(random)
            val gate = random.nextDouble(0.5, 60.0)

            fun attempt(maxJump: Double): CorrectionResult {
                val rig = Rig()
                rig.seed(current)
                return rig.estimator.applyCorrection(
                    measured,
                    timestampNanos = 0L,
                    blend = 1.0,
                    maxJumpInches = maxJump,
                    maxJumpRadians = Double.MAX_VALUE,
                )
            }

            val atGate = attempt(gate)
            if (atGate == CorrectionResult.REJECTED_JUMP) {
                // Rejected at this gate → rejected at any tighter gate.
                assertEquals(CorrectionResult.REJECTED_JUMP, attempt(gate / 2.0))
            } else {
                // Accepted at this gate → accepted at any looser gate.
                assertEquals(CorrectionResult.APPLIED, attempt(gate * 2.0))
            }
        }
    }

    @Test
    fun blendedResultLandsBetweenCurrentAndCorrected() {
        val random = Random(41)
        repeat(300) {
            val current = randomPose(random)
            val measured = randomPose(random)
            val blend = random.nextDouble(0.0, 1.0)

            val rig = Rig()
            rig.seed(current)
            // Same timestamp as the only sample → corrected == measured.
            val result = rig.estimator.applyCorrection(
                measured,
                timestampNanos = 0L,
                blend = blend,
                maxJumpInches = Double.MAX_VALUE,
                maxJumpRadians = Double.MAX_VALUE,
            )
            assertEquals(CorrectionResult.APPLIED, result)

            val applied = rig.pose
            // Each translation axis interpolates exactly.
            assertEquals(current.x + (measured.x - current.x) * blend, applied.x, 1e-9)
            assertEquals(current.y + (measured.y - current.y) * blend, applied.y, 1e-9)
            // Heading moves along the shortest arc, by the blended fraction.
            val fullDelta = shortestAngleDelta(current.heading, measured.heading)
            val appliedDelta = normalizeAngleSigned(applied.heading - current.heading)
            assertEquals(fullDelta * blend, appliedDelta, 1e-9)
            // Never overshoots the measurement.
            assertTrue(abs(appliedDelta) <= abs(fullDelta) + 1e-9)
        }
    }

    @Test
    fun repeatedStreamingCorrectionsConvergeFromAnyStart() {
        val random = Random(5)
        repeat(50) {
            val rig = Rig()
            rig.seed(randomPose(random))
            val truth = randomPose(random)

            repeat(20) { i ->
                rig.clock.now = (i + 1) * 50_000_000L
                rig.estimator.sample(rig.clock.now, rig.pose)
                rig.estimator.applyCorrection(
                    truth,
                    timestampNanos = rig.clock.now,
                    blend = 0.5,
                    maxJumpInches = Double.MAX_VALUE,
                    maxJumpRadians = Double.MAX_VALUE,
                )
            }

            assertTrue(rig.pose.distanceTo(truth) < 0.01)
            assertTrue(abs(shortestAngleDelta(rig.pose.heading, truth.heading)) < 0.01)
        }
    }

    @Test
    fun nonFiniteMeasurementsAreRejectedBeforeTheyCanPoisonThePose() {
        val rig = Rig()
        rig.seed(Pose2d(10.0, 20.0, 1.0))

        for (bad in listOf(
            Pose2d(Double.NaN, 0.0, 0.0),
            Pose2d(0.0, Double.POSITIVE_INFINITY, 0.0),
            Pose2d(0.0, 0.0, Double.NaN),
        )) {
            val result = rig.estimator.applyCorrection(
                bad,
                timestampNanos = 0L,
                blend = 1.0,
                maxJumpInches = Double.MAX_VALUE,
                maxJumpRadians = Double.MAX_VALUE,
            )
            assertEquals(CorrectionResult.REJECTED_JUMP, result)
        }
        assertEquals(10.0, rig.pose.x, 0.0)
        assertEquals(1.0, rig.pose.heading, 0.0)
    }
}
