package org.firstinspires.ftc.teamcode.core.util

import org.firstinspires.ftc.teamcode.core.geometry.FieldSymmetry
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngle
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

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
     * season's field symmetry.
     */
    fun mirror(
        pose: Pose2d,
        symmetry: FieldSymmetry = RobotConfig.Field.SYMMETRY,
        fieldLength: Double = RobotConfig.Field.LENGTH_INCHES,
    ): Pose2d = if (this == RED) pose else pose.mirror(symmetry, fieldLength)

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
            FieldSymmetry.ROTATE -> normalizeAngle(headingRadians + Math.PI)
        }
    }
}
