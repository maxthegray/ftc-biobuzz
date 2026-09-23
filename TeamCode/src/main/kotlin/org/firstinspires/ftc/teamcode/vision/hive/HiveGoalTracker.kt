package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import java.util.EnumMap
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.core.estimation.isFinite
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell

/** One Limelight frame placed on the robot: what was seen, when, and where the turret and robot were. */
data class TagFrame(
    val fiducials: List<LimelightFiducial>,
    /** Estimated exposure time on the robot's monotonic clock. */
    val captureNanos: Long,
    val turretAngleRad: Double,
    /** Field pose at [captureNanos], or null when unknown (bench). */
    val robotPose: Pose? = null,
)

/** One season tag in one frame, in robot coordinates at capture time. */
data class TagRow(
    val tag: BiobuzzAprilTags.Tag,
    val tagRobot: Vec3,
    /** The CELL opening centre this tag alone implies. */
    val goalRobot: Vec3,
    val heightClass: TagHeightClass,
    /** How far the tag face points below horizontal; a cross-check on raised/lowered only. */
    val facingBelowHorizontalDeg: Double,
    /** Distance from [goalRobot] to its CELL's fused goal (or the median when fusion failed). */
    val deviationIn: Double,
    val usedInFusion: Boolean,
)

/** One CELL's goal fused from its tags in one frame. */
data class CellGoal(
    val cell: Cell,
    val goalRobot: Vec3,
    /** Largest distance of a used tag's goal point from [goalRobot]. */
    val spreadIn: Double,
    val tagIds: List<Int>,
    /** Consensus of the used tags; AMBIGUOUS when they disagree or all sit near the threshold. */
    val heightClass: TagHeightClass,
)

/** An alliance's goal as of now, re-projected onto the current pose when possible. */
data class GoalObservation(
    val alliance: Alliance,
    val cell: Cell,
    /** Goal in the current robot frame (capture-time frame if not re-projected). */
    val goalRobot: Vec3,
    val turretBearingRad: Double,
    val robotBearingRad: Double,
    val horizontalDistanceIn: Double,
    val heightIn: Double,
    val spreadIn: Double,
    val tagIds: List<Int>,
    val captureNanos: Long,
    val ageMs: Double,
    val reprojected: Boolean,
)

/**
 * Per-frame HIVE goal estimation for both alliances.
 *
 * Every season tag with a solved 3D pose becomes a [TagRow]: its own position
 * and the goal it implies, in robot coordinates at capture time, and a
 * raised/lowered class from its height. Rows vote on each HIVE's state
 * ([HiveStateEstimator]); a lowered CELL's tags vote as well as a raised
 * CELL's. The goal for an alliance is the fused goal of the CELL its HIVE state
 * says is raised, unless that CELL's tags read LOWERED this frame.
 *
 * With robot poses, the goal is kept in field coordinates and re-projected onto
 * the current pose, so bearings stay right while the robot moves between
 * frames. Goals expire after [HiveGoalSettings.lostTimeoutMs].
 */
class HiveGoalTracker(priors: Map<Alliance, HiveState?> = emptyMap()) {

    val estimators: Map<Alliance, HiveStateEstimator> =
        Alliance.entries.associateWith { HiveStateEstimator(it, priors[it]) }

    var rows: List<TagRow> = emptyList()
        private set

    var cellGoals: Map<Cell, CellGoal> = emptyMap()
        private set

    var framesProcessed: Long = 0
        private set

    private class HeldGoal(val cellGoal: CellGoal, val captureNanos: Long, val fieldX: Double?, val fieldY: Double?)

    private val held = EnumMap<Alliance, HeldGoal>(Alliance::class.java)

    fun update(frame: TagFrame, mount: TurretCameraMount, settings: HiveGoalSettings): List<HiveTransition> {
        framesProcessed++
        val raw = frame.fiducials.mapNotNull { rawRow(it, frame.turretAngleRad, mount, settings) }
        val fused = raw.groupBy { it.tag.cell }.mapValues { (cell, cellRows) -> fuse(cell, cellRows, settings) }
        rows = fused.values.flatMap { it.second }.sortedBy { it.tag.id }
        cellGoals = fused.mapNotNull { (cell, result) -> result.first?.let { cell to it } }.toMap()

        val transitions = ArrayList<HiveTransition>()
        for ((alliance, estimator) in estimators) {
            vote(alliance)?.let { v ->
                estimator.observe(v, frame.captureNanos, settings.tipConfirmFrames)?.let {
                    transitions += it
                    held.remove(alliance)
                }
            }
            val state = estimator.state ?: continue
            val goal = cellGoals[Cell(alliance, state.raised)] ?: continue
            if (goal.heightClass == TagHeightClass.LOWERED) continue
            held[alliance] = HeldGoal(
                goal,
                frame.captureNanos,
                fieldX(frame.robotPose, goal.goalRobot),
                fieldY(frame.robotPose, goal.goalRobot),
            )
        }
        return transitions
    }

    /** The alliance's goal now, or null if never seen or older than the lost timeout. */
    fun goal(
        alliance: Alliance,
        nowNanos: Long,
        nowPose: Pose?,
        mount: TurretCameraMount,
        settings: HiveGoalSettings,
    ): GoalObservation? {
        val h = held[alliance] ?: return null
        val ageMs = (nowNanos - h.captureNanos) / 1e6
        if (ageMs > settings.lostTimeoutMs) return null

        val captured = h.cellGoal.goalRobot
        val reproject = nowPose != null && h.fieldX != null && h.fieldY != null && nowPose.isFinite()
        val point = if (reproject) {
            val dx = h.fieldX!! - nowPose!!.x()
            val dy = h.fieldY!! - nowPose.y()
            val c = cos(nowPose.heading())
            val s = sin(nowPose.heading())
            Vec3(dx * c + dy * s, -dx * s + dy * c, captured.z)
        } else {
            captured
        }
        return GoalObservation(
            alliance = alliance,
            cell = h.cellGoal.cell,
            goalRobot = point,
            turretBearingRad = GoalGeometry.turretBearingRad(point, mount),
            robotBearingRad = GoalGeometry.robotBearingRad(point),
            horizontalDistanceIn = GoalGeometry.horizontalDistanceFromTurretIn(point, mount),
            heightIn = point.z,
            spreadIn = h.cellGoal.spreadIn,
            tagIds = h.cellGoal.tagIds,
            captureNanos = h.captureNanos,
            ageMs = ageMs,
            reprojected = reproject,
        )
    }

    private fun rawRow(f: LimelightFiducial, turretAngleRad: Double, mount: TurretCameraMount, settings: HiveGoalSettings): TagRow? {
        val tag = BiobuzzAprilTags.lookup(f.id) ?: return null
        if (!BiobuzzAprilTags.isSeasonFamily(f.family)) return null
        val pose = f.targetPoseCameraSpace ?: return null
        val tagRobot = GoalGeometry.cameraToRobot(pose.positionInches(), turretAngleRad, mount)
        val goalRobot = GoalGeometry.cameraToRobot(GoalGeometry.goalInCamera(pose, tag), turretAngleRad, mount)
        if (!tagRobot.isFinite() || !goalRobot.isFinite()) return null
        val facing = GoalGeometry.directionCameraToRobot(GoalGeometry.tagFacingInCamera(pose), turretAngleRad, mount)
        return TagRow(
            tag = tag,
            tagRobot = tagRobot,
            goalRobot = goalRobot,
            heightClass = classify(tagRobot.z, settings),
            facingBelowHorizontalDeg = GoalGeometry.degreesBelowHorizontal(facing),
            deviationIn = Double.NaN,
            usedInFusion = false,
        )
    }

    private fun fuse(cell: Cell, cellRows: List<TagRow>, settings: HiveGoalSettings): Pair<CellGoal?, List<TagRow>> {
        val median = Vec3(
            median(cellRows.map { it.goalRobot.x }),
            median(cellRows.map { it.goalRobot.y }),
            median(cellRows.map { it.goalRobot.z }),
        )
        val kept = cellRows.filter { it.goalRobot.distanceTo(median) <= settings.maxTagSpreadIn }
        if (kept.isEmpty()) {
            return null to cellRows.map { it.copy(deviationIn = it.goalRobot.distanceTo(median)) }
        }
        val mean = kept.fold(Vec3(0.0, 0.0, 0.0)) { acc, r -> acc + r.goalRobot } * (1.0 / kept.size)
        val keptIds = kept.map { it.tag.id }.toSet()
        val finalRows = cellRows.map {
            it.copy(deviationIn = it.goalRobot.distanceTo(mean), usedInFusion = it.tag.id in keptIds)
        }
        val goal = CellGoal(
            cell = cell,
            goalRobot = mean,
            spreadIn = kept.maxOf { it.goalRobot.distanceTo(mean) },
            tagIds = kept.map { it.tag.id }.sorted(),
            heightClass = consensus(kept.map { it.heightClass }),
        )
        return goal to finalRows
    }

    /** All decided rows of this alliance must imply the same state; otherwise no vote. */
    private fun vote(alliance: Alliance): HiveState? {
        val votes = rows.filter { it.tag.cell.alliance == alliance }.mapNotNull { row ->
            when (row.heightClass) {
                TagHeightClass.RAISED -> HiveState.withRaised(row.tag.cell.location)
                TagHeightClass.LOWERED -> HiveState.withRaised(row.tag.cell.location).opposite()
                TagHeightClass.AMBIGUOUS -> null
            }
        }.toSet()
        return votes.singleOrNull()
    }

    private fun HiveState.opposite(): HiveState = HiveState.withRaised(lowered)

    private fun classify(heightIn: Double, settings: HiveGoalSettings): TagHeightClass = when {
        heightIn >= settings.raisedMinHeightIn + settings.classificationMarginIn -> TagHeightClass.RAISED
        heightIn <= settings.raisedMinHeightIn - settings.classificationMarginIn -> TagHeightClass.LOWERED
        else -> TagHeightClass.AMBIGUOUS
    }

    private fun consensus(classes: List<TagHeightClass>): TagHeightClass {
        val decided = classes.filter { it != TagHeightClass.AMBIGUOUS }.toSet()
        return decided.singleOrNull() ?: TagHeightClass.AMBIGUOUS
    }

    private fun median(values: List<Double>): Double {
        val sorted = values.sorted()
        val mid = sorted.size / 2
        return if (sorted.size % 2 == 1) sorted[mid] else (sorted[mid - 1] + sorted[mid]) / 2.0
    }

    private fun fieldX(pose: Pose?, goal: Vec3): Double? =
        pose?.takeIf { it.isFinite() }?.let { it.x() + goal.x * cos(it.heading()) - goal.y * sin(it.heading()) }

    private fun fieldY(pose: Pose?, goal: Vec3): Double? =
        pose?.takeIf { it.isFinite() }?.let { it.y() + goal.x * sin(it.heading()) + goal.y * cos(it.heading()) }
}
