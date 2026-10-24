package org.firstinspires.ftc.teamcode.vision

import com.pedropathing.math.Pose
import kotlin.math.atan2
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.subsystems.shortestAngleDelta

/** One AprilTag in one Limelight result, degrees from the crosshair: tx positive right, ty positive up. */
data class TagSighting(val id: Int, val txDeg: Double, val tyDeg: Double)

/** The raised CELL as the camera sees it: the average of its visible tags. */
data class GoalSighting(val tagIds: List<Int>, val txDeg: Double, val tyDeg: Double)

/**
 * Where the turret aims. Each alliance has one HIVE with two CELLs; the HIVE
 * pivots so one CELL faces up, and that raised CELL is the goal. Each CELL has
 * four AprilTags on its bottom face, which face away while it is lowered.
 *
 *  - [goal]: our tags in view mean their CELL is raised: aim at their average
 *    `tx`. If both of our CELLs show, the higher one (larger `ty`) is raised.
 *    Only shoot at a goal.
 *  - [pivotBearingRad]: no goal in view: aim at our HIVE's pivot from the
 *    odometry pose. Both CELLs are within ~16 in. of it, so whichever is
 *    raised comes into view.
 *
 * Tag IDs (manual §9.6, Figure 9-16): RED 30–33 far CELL, 34–37 audience CELL;
 * BLUE 38–41 audience CELL, 42–45 far CELL. The full HIVE estimator (3D goal,
 * HIVE state) is at git tag `vision-full`.
 */
object GoalAim {

    private val CELL_TAG_IDS = mapOf(
        Alliance.RED to listOf(30..33, 34..37),
        Alliance.BLUE to listOf(38..41, 42..45),
    )

    /** HIVE centres 25.5 in. apart at the field centre, RED on the audience's left (Figures 9-10, 9-17). */
    private const val HIVE_OFFSET_FROM_CENTRE_IN = 12.75

    fun goal(alliance: Alliance, tags: List<TagSighting>): GoalSighting? =
        CELL_TAG_IDS.getValue(alliance)
            .map { ids -> tags.filter { it.id in ids } }
            .filter { it.isNotEmpty() }
            .map { seen -> GoalSighting(seen.map { it.id }, seen.map { it.txDeg }.average(), seen.map { it.tyDeg }.average()) }
            .maxByOrNull { it.tyDeg }

    /** [alliance]'s HIVE pivot in Pedro field inches. */
    fun pivot(alliance: Alliance): Pose {
        val centre = RobotConfig.Field.LENGTH_INCHES / 2.0
        return alliance.mirror(Pose(centre - HIVE_OFFSET_FROM_CENTRE_IN, centre, 0.0))
    }

    /** Bearing from the robot's heading to [alliance]'s HIVE pivot: radians, CCW positive, in (-π, π]. */
    fun pivotBearingRad(robot: Pose, alliance: Alliance): Double {
        val p = pivot(alliance)
        return shortestAngleDelta(robot.heading(), atan2(p.y() - robot.y(), p.x() - robot.x()))
    }
}
