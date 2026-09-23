package org.firstinspires.ftc.teamcode.vision.hive

import com.pedropathing.math.Pose
import java.util.EnumMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretAngleSource
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance

/**
 * HIVE state and goal for both alliances from the turret-mounted Limelight.
 * Touches no hardware: it reads [limelight] in `periodic()` and places each new
 * fresh frame on the robot with the turret angle (and, when given, the robot
 * pose) at the frame's estimated capture time.
 *
 * Capture time is now − `estimatedCaptureAgeMs`: first receipt age plus the
 * Limelight's capture and targeting latency, so it still excludes USB
 * transport and the wait before the first poll.
 *
 * Pass [priors] = [MATCH_SETUP] in autonomous. TeleOp starts unknown (auton may
 * have tipped a HIVE) and learns the state from the first confirming frames.
 * [poseAt]/[currentPose] (e.g. `localizer.estimator::poseAt`, `{ localizer.pose }`)
 * keep a held goal right while the robot moves; without them goals stay in the
 * capture-time robot frame.
 */
class HiveGoalSubsystem(
    private val limelight: LimelightSubsystem,
    private val turret: TurretAngleSource,
    priors: Map<Alliance, HiveState?> = emptyMap(),
    private val poseAt: ((Long) -> Pose?)? = null,
    private val currentPose: (() -> Pose?)? = null,
    private val eventSink: (String) -> Unit = {},
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("HiveGoal") {

    override val registerAfter: Class<out SubsystemBase> get() = LimelightSubsystem::class.java

    val tracker = HiveGoalTracker(priors)

    var mount = TurretCameraMount()
        private set
    var settings = HiveGoalSettings()
        private set

    /** Fresh frames dropped because the turret angle at capture time was unknown. */
    var framesWithoutTurretAngle = 0L
        private set

    var newFrameThisTick = false
        private set

    private val goals = EnumMap<Alliance, GoalObservation>(Alliance::class.java)
    private var columns = RowColumns.EMPTY
    private var columnsFrame = -1L

    fun goal(alliance: Alliance): GoalObservation? = goals[alliance]

    fun state(alliance: Alliance): HiveState? = tracker.estimators.getValue(alliance).state

    /** True while the state is the match-setup prior and no frame has confirmed it. */
    fun stateAssumed(alliance: Alliance): Boolean = tracker.estimators.getValue(alliance).assumed

    fun tipCount(alliance: Alliance): Int = tracker.estimators.getValue(alliance).tipCount

    /** Capture time (robot `Clock` nanos) of the frame that confirmed the latest tip. */
    fun lastTipNanos(alliance: Alliance): Long? = tracker.estimators.getValue(alliance).lastTipNanos

    /** True if a frame cast a vote on this HIVE's state within [ms]. */
    fun observedWithinMs(alliance: Alliance, ms: Double): Boolean {
        val seen = tracker.estimators.getValue(alliance).lastObservedNanos ?: return false
        return (clock.nanos() - seen) / 1e6 <= ms
    }

    override fun periodic() {
        mount = TurretCameraMount.fromConfig()
        settings = HiveGoalConfig.snapshot()
        val now = clock.nanos()

        newFrameThisTick = false
        if (limelight.newFrameThisTick && limelight.resultFresh && limelight.pipelineMatches) {
            val captureNanos = now - (limelight.estimatedCaptureAgeMs * 1e6).toLong()
            val turretAngle = turret.angleAtRad(captureNanos)
            if (turretAngle == null || !turretAngle.isFinite()) {
                framesWithoutTurretAngle++
            } else {
                newFrameThisTick = true
                val frame = TagFrame(limelight.fiducials, captureNanos, turretAngle, poseAt?.invoke(captureNanos))
                for (transition in tracker.update(frame, mount, settings)) eventSink(transition.describe())
            }
        }

        val nowPose = currentPose?.invoke()
        for (alliance in Alliance.entries) {
            val goal = tracker.goal(alliance, now, nowPose, mount, settings)
            if (goal == null) goals.remove(alliance) else goals[alliance] = goal
        }
    }

    override fun health(): String {
        val hives = Alliance.entries.joinToString("; ") { alliance ->
            val state = state(alliance)?.let { "${it.raised.name.lowercase(Locale.US)} up${if (stateAssumed(alliance)) " (assumed)" else ""}" }
                ?: "state unknown"
            val goal = goal(alliance)?.let { "goal %.0f in".format(Locale.US, it.horizontalDistanceIn) } ?: "no goal"
            "${alliance.name} $state, $goal"
        }
        return if (mount.measured) hives else "camera mount not measured; $hives"
    }

    override fun logState(log: StateLog) {
        for (alliance in Alliance.entries) {
            val prefix = alliance.name
            val estimator = tracker.estimators.getValue(alliance)
            log.put("$prefix/state", estimator.state?.name ?: "UNKNOWN")
            log.put("$prefix/stateAssumed", estimator.assumed)
            log.put("$prefix/tipCount", estimator.tipCount.toLong())
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
        }
        log.put("frames/processed", tracker.framesProcessed)
        log.put("frames/withoutTurretAngle", framesWithoutTurretAngle)
        if (tracker.framesProcessed != columnsFrame) {
            columnsFrame = tracker.framesProcessed
            columns = RowColumns.of(tracker.rows)
        }
        log.put("tags/id", columns.id)
        log.put("tags/heightIn", columns.heightIn)
        log.put("tags/class", columns.heightClass)
        log.put("tags/goalXIn", columns.goalXIn)
        log.put("tags/goalYIn", columns.goalYIn)
        log.put("tags/goalZIn", columns.goalZIn)
        log.put("tags/deviationIn", columns.deviationIn)
        log.put("tags/used", columns.used)
        log.put("tags/facingBelowHorizontalDeg", columns.facingBelowHorizontalDeg)
        log.put("mount/measured", mount.measured)
        log.put("mount/axisForwardIn", mount.axisForwardIn)
        log.put("mount/axisLeftIn", mount.axisLeftIn)
        log.put("mount/forwardIn", mount.forwardIn)
        log.put("mount/leftIn", mount.leftIn)
        log.put("mount/heightIn", mount.heightIn)
        log.put("mount/pitchUpDeg", mount.pitchUpDeg)
        log.put("mount/yawDeg", mount.yawDeg)
        log.put("settings/raisedMinHeightIn", settings.raisedMinHeightIn)
        log.put("settings/classificationMarginIn", settings.classificationMarginIn)
        log.put("settings/maxTagSpreadIn", settings.maxTagSpreadIn)
    }

    /** Parallel per-tag arrays of the latest frame, rebuilt only when a frame is processed. */
    private class RowColumns(
        val id: DoubleArray,
        val heightIn: DoubleArray,
        /** 1 raised, −1 lowered, 0 ambiguous. */
        val heightClass: DoubleArray,
        val goalXIn: DoubleArray,
        val goalYIn: DoubleArray,
        val goalZIn: DoubleArray,
        val deviationIn: DoubleArray,
        /** 1 when the tag contributed to its CELL's fused goal. */
        val used: DoubleArray,
        val facingBelowHorizontalDeg: DoubleArray,
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
                goalXIn = DoubleArray(rows.size) { rows[it].goalRobot.x },
                goalYIn = DoubleArray(rows.size) { rows[it].goalRobot.y },
                goalZIn = DoubleArray(rows.size) { rows[it].goalRobot.z },
                deviationIn = DoubleArray(rows.size) { rows[it].deviationIn },
                used = DoubleArray(rows.size) { if (rows[it].usedInFusion) 1.0 else 0.0 },
                facingBelowHorizontalDeg = DoubleArray(rows.size) { rows[it].facingBelowHorizontalDeg },
            )
        }
    }

    companion object {
        /** Manual §10.3.1 staging for both HIVEs; use in autonomous. */
        val MATCH_SETUP: Map<Alliance, HiveState?> =
            Alliance.entries.associateWith { HiveState.matchSetup(it) }
    }
}
