package org.firstinspires.ftc.teamcode.vision.hive

import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightPose
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.CellLocation

/**
 * Synthetic Limelight fiducials for a level camera at the robot's pose point
 * on the tiles ([mount]) with the turret at 0, so camera (right, down, out)
 * = robot (−y, −z, x). Every tag squarely faces the lens.
 */
internal object HiveTestFrames {

    val mount = TurretCameraMount()

    /** Goal height of a raised CELL; its tags sit 7.19 in lower, well above the 43.95 in threshold. */
    const val RAISED_GOAL_HEIGHT_IN = 60.0

    /** Goal height of a lowered CELL; its tags sit well below the threshold. */
    const val LOWERED_GOAL_HEIGHT_IN = 36.0

    fun cell(alliance: Alliance, location: CellLocation) = Cell(alliance, location)

    /** Tags of [cell] whose solved poses all imply a goal at [goalRobot]; [ids] limits which are visible. */
    fun cluster(
        cell: Cell,
        goalRobot: Vec3,
        ids: List<Int> = BiobuzzAprilTags.tagsOf(cell).map { it.id },
        perturbIn: Map<Int, Vec3> = emptyMap(),
    ): List<LimelightFiducial> {
        val goalCam = Vec3(-goalRobot.y, -goalRobot.z, goalRobot.x)
        return BiobuzzAprilTags.tagsOf(cell).filter { it.id in ids }.map { tag ->
            val centre = goalCam +
                Vec3(tag.offsetFromClusterCenterInches, BiobuzzAprilTags.TAG_ROW_Y_INCHES, BiobuzzAprilTags.TAG_ROW_Z_INCHES) +
                (perturbIn[tag.id] ?: Vec3(0.0, 0.0, 0.0))
            fiducial(tag.id, centre)
        }
    }

    fun raised(alliance: Alliance, location: CellLocation, forwardIn: Double = 80.0, leftIn: Double = 0.0) =
        cluster(cell(alliance, location), Vec3(forwardIn, leftIn, RAISED_GOAL_HEIGHT_IN))

    fun lowered(alliance: Alliance, location: CellLocation, forwardIn: Double = 80.0, leftIn: Double = 0.0) =
        cluster(cell(alliance, location), Vec3(forwardIn, leftIn, LOWERED_GOAL_HEIGHT_IN))

    fun fiducial(id: Int, centreCamInches: Vec3, family: String = "36H11C") = LimelightFiducial(
        id = id,
        family = family,
        txDegrees = 0.0,
        tyDegrees = 0.0,
        txNoCrosshairDegrees = 0.0,
        tyNoCrosshairDegrees = 0.0,
        areaPercent = 1.0,
        targetPoseCameraSpace = LimelightPose(
            centreCamInches.x * 0.0254,
            centreCamInches.y * 0.0254,
            centreCamInches.z * 0.0254,
            0.0,
            0.0,
            0.0,
        ),
        cameraPoseTargetSpace = null,
        cornerCount = 4,
    )
}
