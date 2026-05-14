package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Fuses camera-derived AprilTag pose estimates into the follower's pose
 * estimate so wheel drift gets bounded over time.
 *
 * Strategy (deliberately conservative — you almost never want a shaky
 * camera yanking the robot's pose estimate mid-motion):
 *
 *  1. Consume [Observation]s from whichever vision source you plug in.
 *  2. Reject anything farther than [maxRangeInches] — long-range AprilTag
 *     pose is noisy and the lens distortion becomes dominant.
 *  3. Require two successive observations whose poses agree within
 *     [stabilityToleranceInches] / [stabilityToleranceRadians] before
 *     committing. This filters out a misdecoded single frame.
 *  4. When committing, write directly to the [Localizer] — a hard snap.
 *     Weighted fusion is left to whoever wants to swap in a Kalman filter.
 *
 * The corrector is a subsystem so Ivy commands can `requiring(aprilTagCorrector)`
 * to pause corrections during precision auton moves.
 */
class AprilTagCorrector(
    private val localizer: Localizer,
    var maxRangeInches: Double = 96.0,
    var stabilityToleranceInches: Double = 1.5,
    var stabilityToleranceRadians: Double = Math.toRadians(4.0),
) : SubsystemBase("AprilTagCorrector") {

    /** One AprilTag-derived field pose, plus the slant range to the tag that produced it. */
    data class Observation(val fieldPose: Pose, val rangeInches: Double, val tagId: Int)

    var enabled: Boolean = true

    private var pendingObservation: Observation? = null
    var lastAppliedAt: Long = 0L
        private set
    var appliedCount: Int = 0
        private set
    var rejectedCount: Int = 0
        private set

    /**
     * Push a fresh observation. Returns true if the observation was accepted
     * and committed to the localizer; false otherwise.
     */
    fun submit(obs: Observation): Boolean {
        if (!enabled) return false

        if (obs.rangeInches > maxRangeInches) {
            rejectedCount++
            pendingObservation = null
            return false
        }

        val prev = pendingObservation
        if (prev == null || prev.tagId != obs.tagId) {
            pendingObservation = obs
            return false
        }

        val dxy = hypot(obs.fieldPose.x - prev.fieldPose.x, obs.fieldPose.y - prev.fieldPose.y)
        val dTheta = abs(normalizeAngle(obs.fieldPose.heading - prev.fieldPose.heading))

        if (dxy > stabilityToleranceInches || dTheta > stabilityToleranceRadians) {
            pendingObservation = obs
            rejectedCount++
            return false
        }

        localizer.setPose(obs.fieldPose)
        lastAppliedAt = System.nanoTime()
        appliedCount++
        pendingObservation = null
        return true
    }

    override fun periodic() {
        // Nothing to do per-tick: corrections are event-driven via submit().
    }

    private fun normalizeAngle(radians: Double): Double {
        var a = radians % (2.0 * Math.PI)
        if (a > Math.PI) a -= 2.0 * Math.PI
        if (a < -Math.PI) a += 2.0 * Math.PI
        return a
    }
}
