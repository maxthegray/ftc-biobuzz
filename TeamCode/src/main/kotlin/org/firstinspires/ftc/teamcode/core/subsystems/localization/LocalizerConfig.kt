package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.bylazar.configurables.annotations.Configurable
import dev.frozenmilk.sinister.loading.Pinned

/**
 * Runtime tuning knobs for external pose corrections (vision, wall snaps)
 * applied through [LocalizerSubsystem.applyCorrection].
 *
 * [Pinned] for the same reason as
 * [org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig]: Panels
 * live-tunes these statics, and an unpinned class would reset them to the
 * compiled defaults on every Sloth hot reload. Trade-off: edits to *this
 * file* need a full install.
 */
@Pinned
@Configurable
object LocalizerConfig {

    private const val DEFAULT_CORRECTION_BLEND = 0.5
    private const val DEFAULT_MAX_CORRECTION_INCHES = 12.0
    private val DEFAULT_MAX_CORRECTION_RADIANS = Math.toRadians(30.0)
    private const val DEFAULT_FOLLOWING_BLEND_SCALE = 0.25

    /**
     * Fraction of each accepted correction applied to the pose (0..1).
     * 1.0 snaps fully onto the measurement; lower values act as a
     * complementary filter for streaming sources — at 0.5 a steady stream
     * converges within ~3 frames while halving single-frame noise. For
     * one-shot corrections (a wall snap), pass `blend = 1.0` at the call
     * site instead of retuning this.
     */
    @JvmField var correctionBlend: Double = DEFAULT_CORRECTION_BLEND

    /** Corrections that would move the pose farther than this are rejected as outliers. */
    @JvmField var maxCorrectionInches: Double = DEFAULT_MAX_CORRECTION_INCHES

    /** Corrections that would rotate the pose more than this (radians) are rejected. */
    @JvmField var maxCorrectionRadians: Double = DEFAULT_MAX_CORRECTION_RADIANS

    /**
     * Extra blend multiplier applied while the drive is actively following a
     * path (0..1). A hard pose snap mid-follow steps Pedro's tracking error
     * and jerks the drive; scaling corrections down keeps streaming vision
     * gentle during paths and lets convergence finish after the path ends.
     * 1.0 disables the policy.
     */
    @JvmField var followingBlendScale: Double = DEFAULT_FOLLOWING_BLEND_SCALE

    internal val safeCorrectionBlend: Double
        get() = if (correctionBlend.isFinite()) {
            correctionBlend.coerceIn(0.0, 1.0)
        } else {
            DEFAULT_CORRECTION_BLEND
        }

    internal val safeMaxCorrectionInches: Double
        get() = finiteAtLeast(maxCorrectionInches, DEFAULT_MAX_CORRECTION_INCHES)

    internal val safeMaxCorrectionRadians: Double
        get() = finiteAtLeast(maxCorrectionRadians, DEFAULT_MAX_CORRECTION_RADIANS)

    internal val safeFollowingBlendScale: Double
        get() = if (followingBlendScale.isFinite()) {
            followingBlendScale.coerceIn(0.0, 1.0)
        } else {
            DEFAULT_FOLLOWING_BLEND_SCALE
        }

    private fun finiteAtLeast(value: Double, fallback: Double): Double =
        if (value.isFinite() && value >= 0.0) value else fallback
}
