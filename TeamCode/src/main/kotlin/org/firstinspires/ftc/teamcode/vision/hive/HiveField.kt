package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation

/**
 * Approximate field position of each raised CELL's opening, for aiming before
 * the Limelight has seen the goal. An aim hint only: never used for
 * localization, and never a reason to shoot.
 *
 * Source: *2026-2027 FIRST Tech Challenge Competition Manual, BIOBUZZ*, V1
 * (September 12, 2026), §9.6, unchanged for these figures through Team Update
 * 02 (September 24, 2026). No coordinates are published, so these are
 * derived from the drawings (manual tolerance ±1 in.):
 *  - §9.6: the HIVE structure is at the centre of the FIELD.
 *  - Figure 9-10: HIVE centre to centre 25.5 in.; Figure 9-17: the red HIVE is
 *    on the audience's left, each HIVE's CELLs on the audience and far sides.
 *  - §9.6.1: pivot axis 43.95 in. above the TILES.
 *  - Figure 9-9: CELLs 18.84 in. apart and 12.04 in. deep along the arm, so a
 *    CELL's outer end face is 21.46 in. from the pivot.
 *  - Figure 9-10: a raised CELL is tilted 30°; its opening (the outer end face)
 *    spans 53.5 to 65.6 in. above the TILES.
 * The opening centre is then 59.55 in. high; solving pivot + 21.46 sin 30° +
 * h cos 30° = 59.55 for the face's offset h puts it 21.46 cos 30° − h sin 30°
 * = 15.8 in. from the pivot, horizontally, towards its own side.
 *
 * Pedro field frame: origin at the corner on the audience's left, +X along the
 * audience wall, +Y away from the audience, inches. Robot frame as in
 * [GoalGeometry]: x forward, y left.
 */
object HiveField {

    const val CENTRE_IN = RobotConfig.Field.LENGTH_INCHES / 2.0
    const val HIVE_CENTRE_TO_CENTRE_IN = 25.5
    const val RAISED_OPENING_FROM_PIVOT_IN = 15.8
    const val RAISED_OPENING_HEIGHT_IN = 59.55

    /** Pivot x of [alliance]'s HIVE. */
    fun pivotX(alliance: Alliance): Double =
        if (alliance == Alliance.RED) CENTRE_IN - HIVE_CENTRE_TO_CENTRE_IN / 2.0 else CENTRE_IN + HIVE_CENTRE_TO_CENTRE_IN / 2.0

    /** Opening centre of [cell] while it is the raised CELL, field inches. */
    fun goal(cell: Cell): Vec3 {
        val towardFar = if (cell.location == CellLocation.FAR) 1.0 else -1.0
        return Vec3(pivotX(cell.alliance), CENTRE_IN + towardFar * RAISED_OPENING_FROM_PIVOT_IN, RAISED_OPENING_HEIGHT_IN)
    }

    /** The CELL on the robot's half of the field. */
    fun cellOnSide(robotY: Double): CellLocation = if (robotY < CENTRE_IN) CellLocation.AUDIENCE else CellLocation.FAR

    /** A field point in the frame of a robot at [pose]; z is unchanged. */
    fun toRobot(field: Vec3, pose: Pose): Vec3 {
        val dx = field.x - pose.x()
        val dy = field.y - pose.y()
        val c = cos(pose.heading())
        val s = sin(pose.heading())
        return Vec3(dx * c + dy * s, -dx * s + dy * c, field.z)
    }

    /** A robot-frame point on the field for a robot at [pose]; z is unchanged. */
    fun toField(robot: Vec3, pose: Pose): Vec3 {
        val c = cos(pose.heading())
        val s = sin(pose.heading())
        return Vec3(pose.x() + robot.x * c - robot.y * s, pose.y() + robot.x * s + robot.y * c, robot.z)
    }
}
