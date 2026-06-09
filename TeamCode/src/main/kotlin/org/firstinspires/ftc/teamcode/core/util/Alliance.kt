package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

/**
 * Alliance / starting-side selection. Used by auton to mirror paths so you
 * write one set of poses per routine and run it on either side.
 *
 * Convention: paths are authored in RED coordinates; [mirror] reflects them
 * across the field's centre line for BLUE. The mirror axis length comes from
 * [RobotConfig.Field.LENGTH_INCHES] so a season with a non-standard field is
 * a one-line edit.
 */
enum class Alliance {
    RED,
    BLUE;

    /**
     * Returns `pose` unchanged for RED; mirrors across the field for BLUE
     * (`x → length − x`, `heading → π − heading`, matching Pedro's
     * `Pose.mirror`).
     */
    fun mirror(pose: Pose): Pose =
        if (this == RED) pose else pose.mirror(RobotConfig.Field.LENGTH_INCHES)

    /**
     * Mirrors a bare heading the same way [mirror] does for a pose's heading.
     * Use this for the radian arguments of `setLinearHeadingInterpolation` /
     * `setConstantHeadingInterpolation` when building both-side paths with
     * Pedro's `PathBuilder` — pose mirroring alone does not cover them.
     */
    fun mirror(headingRadians: Double): Double =
        if (this == RED) headingRadians else Math.PI - headingRadians
}
