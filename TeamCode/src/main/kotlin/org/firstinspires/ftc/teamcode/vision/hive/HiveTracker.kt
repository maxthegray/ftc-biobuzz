package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import java.util.EnumMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.localization.isFinite
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell

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

/** Where a goal came from; only [VISION] is good enough to shoot at. */
enum class GoalSource {
    /** Seen by the Limelight, live or held for up to `lostTimeoutMs`. */
    VISION,

    /** [HiveField]'s approximate point placed with the current pose; an aim hint until tags are seen. */
    ODOMETRY,
}

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
    val source: GoalSource,
)

/**
 * HIVE state and goal for both alliances from the turret-mounted Limelight.
 * Touches no hardware: it reads [limelight] in `periodic()` and places each new
 * fresh frame on the robot with the turret angle (and, when given, the robot
 * pose) at the frame's estimated capture time: now − `estimatedCaptureAgeMs`,
 * which still excludes USB transport and the wait before the first poll.
 *
 * Every season tag with a solved 3D pose becomes a [TagRow]: its position and
 * the goal it implies, and a raised/lowered class from its height. Rows vote on
 * each HIVE's state ([HiveStateEstimator]); a lowered CELL's tags vote as well
 * as a raised CELL's. An alliance's goal is the fused goal of the CELL its HIVE
 * state says is raised, unless that CELL's tags read LOWERED this frame. Goals
 * expire after [HiveConfig.lostTimeoutMs].
 *
 * [turretAngleAt] gives the turret's measured angle (radians, CCW from the
 * robot's front) at a past `Clock` time, never the commanded angle; null when
 * unknown, which drops the frame. Pass [priors] = [MATCH_SETUP] in autonomous;
 * teleop starts unknown (auton may have tipped a HIVE). [poseAt]/[currentPose]
 * (e.g. `localizer.estimator::poseAt`, `{ localizer.pose }`) keep a held goal in
 * field coordinates so bearings stay right while the robot moves.
 *
 * [aimGoal] is what aiming reads: the vision goal when there is one, otherwise
 * [HiveField]'s point for the CELL this tracker believes is raised (or, with
 * the state unknown, the CELL on the robot's half of the field), placed with
 * the current pose. Nothing about past sightings is kept beyond the held goal.
 */
class HiveTracker(
    private val limelight: LimelightSubsystem,
    private val turretAngleAt: (Long) -> Double?,
    priors: Map<Alliance, HiveState?> = emptyMap(),
    private val poseAt: ((Long) -> Pose?)? = null,
    private val currentPose: (() -> Pose?)? = null,
    private val eventSink: (String) -> Unit = {},
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Hive") {

    override val registerAfter: Class<out SubsystemBase> get() = LimelightSubsystem::class.java

    val hives: Map<Alliance, HiveStateEstimator> =
        Alliance.entries.associateWith { HiveStateEstimator(it, priors[it]) }

    /** Season tags of the newest processed frame. */
    var rows: List<TagRow> = emptyList()
        private set

    /** Fused goal of every CELL in the newest processed frame. */
    var cellGoals: Map<Cell, CellGoal> = emptyMap()
        private set

    var framesProcessed: Long = 0
        private set

    /** Fresh frames dropped because the turret angle at capture time was unknown. */
    var framesWithoutTurretAngle = 0L
        private set

    var newFrameThisTick = false
        private set

    /** Estimated capture time of the newest processed frame. */
    var lastFrameCaptureNanos: Long? = null
        private set

    private class HeldGoal(val cellGoal: CellGoal, val captureNanos: Long, val field: Vec3?)

    private val held = EnumMap<Alliance, HeldGoal>(Alliance::class.java)
    private val goals = EnumMap<Alliance, GoalObservation>(Alliance::class.java)
    private val aimGoals = EnumMap<Alliance, GoalObservation>(Alliance::class.java)
    private var columns = RowColumns.EMPTY
    private var columnsFrame = -1L

    /** The vision goal: seen this frame or held for up to `lostTimeoutMs`. */
    fun goal(alliance: Alliance): GoalObservation? = goals[alliance]

    /** [goal], or else the [GoalSource.ODOMETRY] estimate; null only without a current pose. */
    fun aimGoal(alliance: Alliance): GoalObservation? = aimGoals[alliance]

    fun state(alliance: Alliance): HiveState? = hives.getValue(alliance).state

    /** True while the state is the match-setup prior and no frame has confirmed it. */
    fun stateAssumed(alliance: Alliance): Boolean = hives.getValue(alliance).assumed

    fun tipCount(alliance: Alliance): Int = hives.getValue(alliance).tipCount

    /** Capture time (robot `Clock` nanos) of the frame that confirmed the latest tip. */
    fun lastTipNanos(alliance: Alliance): Long? = hives.getValue(alliance).lastTipNanos

    /** True if a frame cast a vote on this HIVE's state within [ms]. */
    fun observedWithinMs(alliance: Alliance, ms: Double): Boolean {
        val seen = hives.getValue(alliance).lastObservedNanos ?: return false
        return (clock.nanos() - seen) / 1e6 <= ms
    }

    override fun periodic() {
        val now = clock.nanos()
        newFrameThisTick = false
        if (limelight.newFrameThisTick && limelight.resultFresh && limelight.pipelineMatches) {
            val captureNanos = now - (limelight.estimatedCaptureAgeMs * 1e6).toLong()
            val turretAngle = turretAngleAt(captureNanos)
            if (turretAngle == null || !turretAngle.isFinite()) {
                framesWithoutTurretAngle++
            } else {
                newFrameThisTick = true
                lastFrameCaptureNanos = captureNanos
                process(limelight.fiducials, captureNanos, turretAngle, poseAt?.invoke(captureNanos))
            }
        }

        val nowPose = currentPose?.invoke()?.takeIf { it.isFinite() }
        for (alliance in Alliance.entries) {
            val goal = project(alliance, now, nowPose)
            if (goal == null) goals.remove(alliance) else goals[alliance] = goal
            val aim = goal ?: nowPose?.let { odometryGoal(alliance, now, it) }
            if (aim == null) aimGoals.remove(alliance) else aimGoals[alliance] = aim
        }
    }

    private fun process(fiducials: List<LimelightFiducial>, captureNanos: Long, turretAngleRad: Double, pose: Pose?) {
        framesProcessed++
        val fused = fiducials.mapNotNull { rawRow(it, turretAngleRad) }
            .groupBy { it.tag.cell }
            .mapValues { (cell, cellRows) -> fuse(cell, cellRows) }
        rows = fused.values.flatMap { it.second }.sortedBy { it.tag.id }
        cellGoals = fused.mapNotNull { (cell, result) -> result.first?.let { cell to it } }.toMap()

        for ((alliance, hive) in hives) {
            vote(alliance)?.let { v ->
                hive.observe(v, captureNanos, HiveConfig.tipConfirmFrames)?.let {
                    eventSink(it.describe())
                    held.remove(alliance)
                }
            }
            val state = hive.state ?: continue
            val goal = cellGoals[Cell(alliance, state.raised)] ?: continue
            if (goal.heightClass == TagHeightClass.LOWERED) continue
            val field = pose?.takeIf { it.isFinite() }?.let { HiveField.toField(goal.goalRobot, it) }
            held[alliance] = HeldGoal(goal, captureNanos, field)
        }
    }

    private fun project(alliance: Alliance, nowNanos: Long, nowPose: Pose?): GoalObservation? {
        val h = held[alliance] ?: return null
        val ageMs = (nowNanos - h.captureNanos) / 1e6
        if (ageMs > HiveConfig.lostTimeoutMs) return null

        val captured = h.cellGoal.goalRobot
        val field = h.field
        val reproject = nowPose != null && field != null
        val point = if (reproject) HiveField.toRobot(field!!, nowPose!!) else captured
        return GoalObservation(
            alliance = alliance,
            cell = h.cellGoal.cell,
            goalRobot = point,
            turretBearingRad = GoalGeometry.turretBearingRad(point),
            robotBearingRad = GoalGeometry.robotBearingRad(point),
            horizontalDistanceIn = GoalGeometry.horizontalDistanceFromTurretIn(point),
            heightIn = point.z,
            spreadIn = h.cellGoal.spreadIn,
            tagIds = h.cellGoal.tagIds,
            captureNanos = h.captureNanos,
            ageMs = ageMs,
            reprojected = reproject,
            source = GoalSource.VISION,
        )
    }

    private fun odometryGoal(alliance: Alliance, nowNanos: Long, pose: Pose): GoalObservation {
        val location = state(alliance)?.raised ?: HiveField.cellOnSide(pose.y())
        val cell = Cell(alliance, location)
        val point = HiveField.toRobot(HiveField.goal(cell), pose)
        return GoalObservation(
            alliance = alliance,
            cell = cell,
            goalRobot = point,
            turretBearingRad = GoalGeometry.turretBearingRad(point),
            robotBearingRad = GoalGeometry.robotBearingRad(point),
            horizontalDistanceIn = GoalGeometry.horizontalDistanceFromTurretIn(point),
            heightIn = point.z,
            spreadIn = Double.NaN,
            tagIds = emptyList(),
            captureNanos = nowNanos,
            ageMs = 0.0,
            reprojected = false,
            source = GoalSource.ODOMETRY,
        )
    }

    private fun rawRow(f: LimelightFiducial, turretAngleRad: Double): TagRow? {
        val tag = BiobuzzAprilTags.lookup(f.id) ?: return null
        if (!BiobuzzAprilTags.isSeasonFamily(f.family)) return null
        val pose = f.targetPoseCameraSpace ?: return null
        val tagRobot = GoalGeometry.cameraToRobot(pose.positionInches(), turretAngleRad)
        val goalRobot = GoalGeometry.cameraToRobot(GoalGeometry.goalInCamera(pose, tag), turretAngleRad)
        if (!tagRobot.isFinite() || !goalRobot.isFinite()) return null
        val facing = GoalGeometry.directionCameraToRobot(GoalGeometry.tagFacingInCamera(pose), turretAngleRad)
        return TagRow(
            tag = tag,
            tagRobot = tagRobot,
            goalRobot = goalRobot,
            heightClass = classify(tagRobot.z),
            facingBelowHorizontalDeg = GoalGeometry.degreesBelowHorizontal(facing),
            deviationIn = Double.NaN,
            usedInFusion = false,
        )
    }

    private fun fuse(cell: Cell, cellRows: List<TagRow>): Pair<CellGoal?, List<TagRow>> {
        val median = Vec3(
            median(cellRows.map { it.goalRobot.x }),
            median(cellRows.map { it.goalRobot.y }),
            median(cellRows.map { it.goalRobot.z }),
        )
        val kept = cellRows.filter { it.goalRobot.distanceTo(median) <= HiveConfig.maxTagSpreadIn }
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
                TagHeightClass.LOWERED -> HiveState.withLowered(row.tag.cell.location)
                TagHeightClass.AMBIGUOUS -> null
            }
        }.toSet()
        return votes.singleOrNull()
    }

    private fun classify(heightIn: Double): TagHeightClass = when {
        heightIn >= HiveConfig.raisedMinHeightIn + HiveConfig.classificationMarginIn -> TagHeightClass.RAISED
        heightIn <= HiveConfig.raisedMinHeightIn - HiveConfig.classificationMarginIn -> TagHeightClass.LOWERED
        else -> TagHeightClass.AMBIGUOUS
    }

    private fun consensus(classes: List<TagHeightClass>): TagHeightClass =
        classes.filter { it != TagHeightClass.AMBIGUOUS }.toSet().singleOrNull() ?: TagHeightClass.AMBIGUOUS

    private fun median(values: List<Double>): Double {
        val sorted = values.sorted()
        val mid = sorted.size / 2
        return if (sorted.size % 2 == 1) sorted[mid] else (sorted[mid - 1] + sorted[mid]) / 2.0
    }

    override fun health(): String {
        val summary = Alliance.entries.joinToString("; ") { alliance ->
            val state = state(alliance)?.let { "${it.raised.name.lowercase(Locale.US)} up${if (stateAssumed(alliance)) " (assumed)" else ""}" }
                ?: "state unknown"
            val goal = goal(alliance)?.let { "goal %.0f in".format(Locale.US, it.horizontalDistanceIn) } ?: "no goal"
            "${alliance.name} $state, $goal"
        }
        return if (HiveConfig.mountMeasured) summary else "camera mount not measured; $summary"
    }

    override fun logState(log: StateLog) {
        for (alliance in Alliance.entries) {
            val prefix = alliance.name
            val hive = hives.getValue(alliance)
            log.put("$prefix/state", hive.state?.name ?: "UNKNOWN")
            log.put("$prefix/stateAssumed", hive.assumed)
            log.put("$prefix/tipCount", hive.tipCount.toLong())
            val goal = goals[alliance]
            log.put("$prefix/visible", goal != null)
            log.put("$prefix/cell", goal?.cell?.location?.name ?: "")
            log.put("$prefix/turretBearingDeg", goal?.let { Math.toDegrees(it.turretBearingRad) } ?: Double.NaN)
            log.put("$prefix/robotBearingDeg", goal?.let { Math.toDegrees(it.robotBearingRad) } ?: Double.NaN)
            log.put("$prefix/distanceIn", goal?.horizontalDistanceIn ?: Double.NaN)
            log.put("$prefix/heightIn", goal?.heightIn ?: Double.NaN)
            log.put("$prefix/spreadIn", goal?.spreadIn ?: Double.NaN)
            log.put("$prefix/ageMs", goal?.ageMs ?: Double.NaN)
            log.put("$prefix/tagIds", goal?.tagIds?.joinToString(",") ?: "")
            log.put("$prefix/reprojected", goal?.reprojected ?: false)
            val aim = aimGoals[alliance]
            log.put("$prefix/aimSource", aim?.source?.name ?: "")
            log.put("$prefix/aimTurretBearingDeg", aim?.let { Math.toDegrees(it.turretBearingRad) } ?: Double.NaN)
            log.put("$prefix/aimRobotBearingDeg", aim?.let { Math.toDegrees(it.robotBearingRad) } ?: Double.NaN)
            val seen = held[alliance]?.takeIf { goal != null }
            val seenField = seen?.field
            log.put("$prefix/fieldGoalErrorIn", seenField?.distanceTo(HiveField.goal(seen.cellGoal.cell)) ?: Double.NaN)
        }
        log.put("frames/processed", framesProcessed)
        log.put("frames/withoutTurretAngle", framesWithoutTurretAngle)
        if (framesProcessed != columnsFrame) {
            columnsFrame = framesProcessed
            columns = RowColumns.of(rows)
        }
        log.put("tags/id", columns.id)
        log.put("tags/heightIn", columns.heightIn)
        log.put("tags/class", columns.heightClass)
        log.put("tags/deviationIn", columns.deviationIn)
        log.put("mount/measured", HiveConfig.mountMeasured)
        log.put("mount/axisForwardIn", HiveConfig.axisForwardIn)
        log.put("mount/axisLeftIn", HiveConfig.axisLeftIn)
        log.put("mount/cameraForwardIn", HiveConfig.cameraForwardIn)
        log.put("mount/cameraLeftIn", HiveConfig.cameraLeftIn)
        log.put("mount/cameraHeightIn", HiveConfig.cameraHeightIn)
        log.put("mount/cameraPitchUpDeg", HiveConfig.cameraPitchUpDeg)
        log.put("mount/cameraYawDeg", HiveConfig.cameraYawDeg)
        log.put("settings/raisedMinHeightIn", HiveConfig.raisedMinHeightIn)
    }

    /** Parallel per-tag arrays of the newest frame, rebuilt only when a frame is processed. */
    private class RowColumns(
        val id: DoubleArray,
        val heightIn: DoubleArray,
        /** 1 raised, −1 lowered, 0 ambiguous. */
        val heightClass: DoubleArray,
        val deviationIn: DoubleArray,
    ) {
        companion object {
            val EMPTY = of(emptyList())

            fun of(rows: List<TagRow>) = RowColumns(
                id = DoubleArray(rows.size) { rows[it].tag.id.toDouble() },
                heightIn = DoubleArray(rows.size) { rows[it].tagRobot.z },
                heightClass = DoubleArray(rows.size) {
                    when (rows[it].heightClass) {
                        TagHeightClass.RAISED -> 1.0
                        TagHeightClass.LOWERED -> -1.0
                        TagHeightClass.AMBIGUOUS -> 0.0
                    }
                },
                deviationIn = DoubleArray(rows.size) { rows[it].deviationIn },
            )
        }
    }

    companion object {
        /** Manual §10.3.1 staging for both HIVEs; use in autonomous. */
        val MATCH_SETUP: Map<Alliance, HiveState?> = Alliance.entries.associateWith { HiveState.matchSetup(it) }
    }
}
