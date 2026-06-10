package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

/**
 * Alliance / starting-side selection. Used by auton to mirror paths automatically
 * so you write one path file per routine and run it on either side.
 */
enum class Alliance {
    RED,
    BLUE;

    /**
     * Returns `pose` unchanged for RED; mirrors across the field for BLUE so a
     * single set of path poses can run on either alliance. The mirror axis is
     * the configured FTC field length in Pedro coordinates.
     */
    fun mirror(pose: Pose): Pose =
        if (this == RED) pose else pose.mirror(RobotConfig.Field.LENGTH_INCHES)

    /**
     * Mirrors a bare heading the same way [mirror] does for a pose's heading
     * (`heading → π − heading`, matching Pedro's `Pose.mirror`). Use this for
     * the radian arguments of `setLinearHeadingInterpolation` /
     * `setConstantHeadingInterpolation` and for `turnTo` targets when
     * building both-side paths — pose mirroring alone does not cover them.
     */
    fun mirror(headingRadians: Double): Double =
        if (this == RED) headingRadians else Math.PI - headingRadians
}
