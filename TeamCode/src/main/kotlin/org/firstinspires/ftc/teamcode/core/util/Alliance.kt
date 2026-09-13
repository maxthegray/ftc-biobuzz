package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.utils.Angle
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

/**
 * How the season's field maps RED coordinates onto BLUE. FTC alternates
 * between the two from game to game, so this is per-season configuration
 * ([RobotConfig.Field.SYMMETRY]).
 */
enum class FieldSymmetry {
    /** Reflection across the field's vertical centreline: (L−x, y, π−h). Pedro 2's `Pose.mirror`. */
    MIRROR,

    /** 180° rotation about the field centre: (L−x, L−y, h+π). Assumes a square field. */
    ROTATE,
}

/**
 * Alliance side. Autonomous poses are written once in RED coordinates and
 * transformed onto BLUE with the season's [FieldSymmetry] and
 * [RobotConfig.Field.LENGTH_INCHES].
 *
 * Use [poses] instead of Pedro's `PoseFactory.mirrorX`: in Pedro 3.0.0
 * `mirrorX` maps heading to −h, which is not this repo's field reflection
 * (π−h) and turns the robot around.
 */
enum class Alliance {
    RED,
    BLUE;

    /** [pose] unchanged for RED; mapped onto the BLUE side for BLUE. */
    fun mirror(
        pose: Pose,
        symmetry: FieldSymmetry = RobotConfig.Field.SYMMETRY,
        fieldLength: Double = RobotConfig.Field.LENGTH_INCHES,
    ): Pose = if (this == RED) {
        pose
    } else {
        when (symmetry) {
            FieldSymmetry.MIRROR -> Pose(fieldLength - pose.x(), pose.y(), Math.PI - pose.heading())
            FieldSymmetry.ROTATE -> Pose(fieldLength - pose.x(), fieldLength - pose.y(), pose.heading() + Math.PI)
        }
    }

    /** A bare heading (e.g. a turn target) transformed the same way as a pose's heading, in [0, 2π). */
    fun mirror(
        headingRadians: Double,
        symmetry: FieldSymmetry = RobotConfig.Field.SYMMETRY,
    ): Double = when {
        this == RED -> headingRadians
        symmetry == FieldSymmetry.MIRROR -> Angle.normalize(Math.PI - headingRadians)
        else -> Angle.normalize(headingRadians + Math.PI)
    }

    /**
     * A Pedro [PoseFactory] (headings in degrees) whose poses are written in
     * RED coordinates and come out on this alliance's side.
     */
    fun poses(): PoseFactory = PoseFactory.degrees().map { mirror(it) }
}
