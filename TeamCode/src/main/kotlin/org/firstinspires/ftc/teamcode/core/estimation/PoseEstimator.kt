package org.firstinspires.ftc.teamcode.core.estimation

import java.util.Locale
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngle
import org.firstinspires.ftc.teamcode.core.geometry.shortestAngleDelta
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerConfig
import org.firstinspires.ftc.teamcode.core.subsystems.localization.PoseHistory
import org.firstinspires.ftc.teamcode.core.util.Clock

/** Outcome of a [PoseEstimator.applyCorrection] attempt. Rejections are logged via the event sink. */
enum class CorrectionResult {
    APPLIED,

    /** Measurement older than `maxAgeNanos` (or timestamped in the future). */
    STALE,

    /** Timestamp outside the retained pose-history window. */
    NO_HISTORY,

    /** Correction larger than the configured outlier gates. */
    REJECTED_JUMP;

    val accepted: Boolean get() = this == APPLIED
}

/**
 * Latency-compensated external-correction filter over a dead-reckoned pose.
 *
 * The primary localizer (Pinpoint via Pedro) is read elsewhere; this class
 * owns the *correction* side: a timestamped [PoseHistory] (sampled once per
 * tick at the moment the pose was measured), and [applyCorrection], which
 * takes a delayed field-pose measurement, re-applies the motion that
 * happened since the measurement, gates the result against outlier limits,
 * and blends it into the current pose.
 *
 * Per-measurement axis weights make partial corrections first-class:
 * an AprilTag at long range has good heading and noisy translation
 * (`translationWeight < 1`), a wall snap knows one axis and heading but
 * not the other axis. A weight of 0 skips both the application *and the
 * outlier gate* for that axis — a measurement that doesn't claim to know
 * the heading can't be rejected for a bogus heading.
 *
 * While the drive is actively following a path ([isFollowing]), accepted
 * corrections are additionally scaled by
 * [LocalizerConfig.followingBlendScale]: a hard `setPose` mid-follow steps
 * the controller's tracking error and jerks the drive, so streaming
 * corrections are fed in gently and convergence finishes after the path.
 */
class PoseEstimator(
    private val currentPose: () -> Pose2d,
    private val applyPose: (Pose2d) -> Unit,
    private val clock: Clock = Clock.SYSTEM,
    private val onEvent: (String) -> Unit = {},
    private val isFollowing: () -> Boolean = { false },
) {
    private val history = PoseHistory()

    /** Record the pose measured this tick. Call once per tick, timestamped at measurement time. */
    fun sample(timestampNanos: Long, pose: Pose2d) {
        history.add(timestampNanos, pose)
    }

    /** The historical pose at [timestampNanos], if it is inside the retained window. */
    fun poseAt(timestampNanos: Long): Pose2d? = history.lookup(timestampNanos)

    /**
     * Apply a delayed field-pose measurement while preserving motion since
     * [timestampNanos]. Gated (rejected as an outlier past [maxJumpInches] /
     * [maxJumpRadians], per axis with non-zero weight) and blended ([blend]
     * fraction per call, scaled per-axis by [translationWeight] /
     * [headingWeight], and by the following policy). Every rejection is
     * recorded via the event sink so the flight log shows why a correction
     * didn't take.
     */
    fun applyCorrection(
        measured: Pose2d,
        timestampNanos: Long,
        maxAgeNanos: Long = 500_000_000,
        blend: Double = LocalizerConfig.safeCorrectionBlend,
        maxJumpInches: Double = LocalizerConfig.safeMaxCorrectionInches,
        maxJumpRadians: Double = LocalizerConfig.safeMaxCorrectionRadians,
        translationWeight: Double = 1.0,
        headingWeight: Double = 1.0,
    ): CorrectionResult {
        // Guard before the jump gate: NaN compares false against any limit,
        // so a non-finite measurement would otherwise sail through the gate
        // and poison the pose.
        if (!measured.x.isFinite() || !measured.y.isFinite() || !measured.heading.isFinite()) {
            onEvent("pose correction rejected: non-finite measurement $measured")
            return CorrectionResult.REJECTED_JUMP
        }
        val age = clock.nanos() - timestampNanos
        if (age < 0 || age > maxAgeNanos) {
            onEvent("pose correction rejected: stale (age ${age / 1_000_000} ms)")
            return CorrectionResult.STALE
        }
        val historical = history.lookup(timestampNanos)
        if (historical == null) {
            onEvent("pose correction rejected: timestamp outside pose history")
            return CorrectionResult.NO_HISTORY
        }
        val current = currentPose()
        val motionSinceMeasurement = current.relativeTo(historical)
        val corrected = measured.transformBy(motionSinceMeasurement)

        val wTranslation = translationWeight.coerceIn(0.0, 1.0)
        val wHeading = headingWeight.coerceIn(0.0, 1.0)
        val jumpInches = corrected.distanceTo(current)
        val headingDelta = shortestAngleDelta(current.heading, corrected.heading)
        val translationRejected = wTranslation > 0.0 && jumpInches > maxJumpInches
        val headingRejected = wHeading > 0.0 && abs(headingDelta) > maxJumpRadians
        if (translationRejected || headingRejected) {
            onEvent(
                "pose correction rejected: jump %.1f in / %.1f deg exceeds gate".format(
                    Locale.US, jumpInches, Math.toDegrees(abs(headingDelta)),
                ),
            )
            return CorrectionResult.REJECTED_JUMP
        }

        val followScale = if (isFollowing()) LocalizerConfig.safeFollowingBlendScale else 1.0
        val b = blend.coerceIn(0.0, 1.0) * followScale
        val bTranslation = b * wTranslation
        val bHeading = b * wHeading
        applyPose(
            Pose2d(
                current.x + (corrected.x - current.x) * bTranslation,
                current.y + (corrected.y - current.y) * bTranslation,
                normalizeAngle(current.heading + headingDelta * bHeading),
            ),
        )
        onEvent(
            "pose correction applied: %.1f in / %.1f deg (blend %.2f, w=%.2f/%.2f%s)".format(
                Locale.US,
                jumpInches,
                Math.toDegrees(abs(headingDelta)),
                b,
                wTranslation,
                wHeading,
                if (followScale < 1.0) ", following" else "",
            ),
        )
        return CorrectionResult.APPLIED
    }
}
