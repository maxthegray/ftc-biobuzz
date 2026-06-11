package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

/**
 * How the season's field maps RED coordinates onto BLUE. FTC alternates
 * between the two from game to game, so this is per-season configuration
 * ([RobotConfig.Field.SYMMETRY]) — using MIRROR in a rotationally symmetric
 * season silently produces wrong BLUE paths.
 */
enum class FieldSymmetry {
    /** Reflection across the field's vertical centerline: (L−x, y, π−h). */
    MIRROR,

    /** 180° rotation about the field center: (L−x, L−y, h+π). Assumes a square field. */
    ROTATE,
}

/**
 * Alliance / starting-side selection. Used by auton to transform RED-coordinate
 * paths onto the BLUE side automatically, so you write one path file per
 * routine and run it on either side. The transform is the season's
 * [FieldSymmetry] from [RobotConfig.Field.SYMMETRY].
 */
enum class Alliance {
    RED,
    BLUE;

    /**
     * Returns `pose` unchanged for RED; maps it onto the BLUE side per the
     * season's field symmetry. MIRROR matches Pedro's `Pose.mirror(length)`.
     */
    fun mirror(
        pose: Pose,
        symmetry: FieldSymmetry = RobotConfig.Field.SYMMETRY,
        fieldLength: Double = RobotConfig.Field.LENGTH_INCHES,
    ): Pose {
        if (this == RED) return pose
        return when (symmetry) {
            FieldSymmetry.MIRROR -> pose.mirror(fieldLength)
            FieldSymmetry.ROTATE -> Pose(
                fieldLength - pose.x,
                fieldLength - pose.y,
                MathFunctions.normalizeAngle(pose.heading + Math.PI),
            )
        }
    }

    /**
     * Transforms a bare heading the same way [mirror] does for a pose's
     * heading. Use this for the radian arguments of
     * `setLinearHeadingInterpolation` / `setConstantHeadingInterpolation` and
     * for `turnTo` targets when building both-side paths — pose mirroring
     * alone does not cover them.
     */
    fun mirror(
        headingRadians: Double,
        symmetry: FieldSymmetry = RobotConfig.Field.SYMMETRY,
    ): Double {
        if (this == RED) return headingRadians
        return when (symmetry) {
            FieldSymmetry.MIRROR -> Math.PI - headingRadians
            FieldSymmetry.ROTATE -> headingRadians + Math.PI
        }
    }
}
