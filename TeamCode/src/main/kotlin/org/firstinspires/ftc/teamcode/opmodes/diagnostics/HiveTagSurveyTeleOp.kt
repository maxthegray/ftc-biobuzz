package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale
import java.util.TreeMap
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.VisionDiagnostics.fmt
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.apriltags.TagSightingTracker
import org.firstinspires.ftc.teamcode.vision.diagnostics.SettingsChangeLog
import org.firstinspires.ftc.teamcode.vision.hive.CellGoal
import org.firstinspires.ftc.teamcode.vision.hive.GoalGeometry
import org.firstinspires.ftc.teamcode.vision.hive.HiveConfig
import org.firstinspires.ftc.teamcode.vision.hive.HiveTracker
import org.firstinspires.ftc.teamcode.vision.hive.TagRow
import org.firstinspires.ftc.teamcode.vision.hive.Vec3

/**
 * Stationary survey of the HIVE tags with the Limelight on a tripod (turret
 * angle held at 0; the `hive` camera fields describe the tripod). For every tag
 * of both alliances it shows the tag's height, raised/lowered class, the CELL
 * opening centre it implies and how far that is from its CELL's fused goal,
 * plus each HIVE's inferred state. Four tags of one CELL agreeing within about
 * an inch checks the camera mount, FIRST's cluster geometry and the Limelight's
 * rotation convention together. Commands no motors.
 *
 * Driver A (after START) saves a lab record of everything seen this run;
 * B clears the survey memory.
 */
@TeleOp(name = "Hive Tag Survey", group = "Diagnostics")
class HiveTagSurveyTeleOp : OpModeBase() {

    private lateinit var startup: VisionDiagnostics.Startup
    private lateinit var limelight: LimelightSubsystem
    private lateinit var hive: HiveTracker
    private lateinit var recorder: VisionLabRecorder
    private val survey = HiveSurveyMemory()
    private val sightings = TagSightingTracker()

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(VisionDiagnostics.limelightRequirement)
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        ConfigStore.register("hive", HiveConfig, HiveConfig::resetDefaults)
        startup = VisionDiagnostics.registerConfigsAndLoad()
        robot.recordEvent(SettingsChangeLog.describe("hive", HiveConfig))
        limelight = robot.register(VisionDiagnostics.limelight(startup))
        hive = robot.register(HiveTracker(limelight, { 0.0 }, eventSink = robot::recordEvent, clock = robot.clock))
        recorder = robot.register(
            VisionLabRecorder("Hive Tag Survey", extraSections = listOf("hive" to HiveConfig.compiledDefaults())),
        )
        driver.button(Button.A).onTrue(instant { recorder.save(survey.measurements(hive, robot.clock.nanos())) })
        driver.button(Button.B).onTrue(
            instant {
                survey.reset()
                robot.recordEvent("Hive Tag Survey memory cleared")
            },
        )
    }

    override fun onStart() {
        robot.recordEvent("Hive Tag Survey started; mount measured=${HiveConfig.mountMeasured}")
    }

    override fun onInitLoop() = tick()

    override fun onLoop() = tick()

    private fun tick() {
        val now = robot.clock.nanos()
        if (limelight.newFrameThisTick && limelight.resultFresh && limelight.pipelineMatches) {
            sightings.recordFrame(limelight.fiducials, now)
        }
        if (hive.newFrameThisTick) hive.lastFrameCaptureNanos?.let { survey.record(hive, it) }
        recorder.poll(robot)

        VisionDiagnostics.restartSection(telemetryBag, startup, null)
        VisionDiagnostics.limelightSections(telemetryBag, limelight, sightings, now, detailed = false)
        telemetryBag.section("Survey Setup") {
            put("mount", if (HiveConfig.mountMeasured) "measured" else "NOT MEASURED: heights and raised/lowered are meaningless")
            put(
                "lens",
                "height ${fmt(HiveConfig.cameraHeightIn, 1)} in, pitch up ${fmt(HiveConfig.cameraPitchUpDeg, 1)}°, " +
                    "yaw ${fmt(HiveConfig.cameraYawDeg, 1)}°, fwd ${fmt(HiveConfig.cameraForwardIn, 1)} / " +
                    "left ${fmt(HiveConfig.cameraLeftIn, 1)} in from the turret axis",
            )
            put("turret", "held at 0° (tripod)")
            put("raised if tag above", "${fmt(HiveConfig.raisedMinHeightIn, 1)} ± ${fmt(HiveConfig.classificationMarginIn, 1)} in")
        }
        telemetryBag.section("HIVEs") {
            for (alliance in Alliance.entries) {
                val state = hive.state(alliance)?.let {
                    "${it.raised.name.lowercase(Locale.US)} CELL raised${if (hive.stateAssumed(alliance)) " (assumed)" else ""}"
                } ?: "unknown"
                val goal = hive.goal(alliance)?.let {
                    "goal ${fmt(it.horizontalDistanceIn, 1)} in at ${fmt(Math.toDegrees(it.turretBearingRad), 1)}°, " +
                        "height ${fmt(it.heightIn, 1)} in, spread ${fmt(it.spreadIn, 2)} in, tags ${it.tagIds.joinToString(",")}"
                } ?: "no goal"
                put(alliance.name, "$state; tips ${hive.tipCount(alliance)}; $goal")
            }
        }
        telemetryBag.section("CELL Goals (latest)") {
            for (cell in BiobuzzAprilTags.cells) {
                val seen = survey.cell(cell)
                put(cell.stickerLabel, seen?.let { describeCell(it.value) + "; ${ago(now, it.captureNanos)}" } ?: "not seen")
            }
        }
        telemetryBag.section("Tags (latest)") {
            for (tag in BiobuzzAprilTags.tags) {
                val seen = survey.tag(tag.id) ?: continue
                put("ID ${tag.id}", describeTag(seen.value) + "; ${ago(now, seen.captureNanos)}")
            }
        }
        telemetryBag.section("Procedure") {
            put("1", "Tripod: measure lens height, pitch up; set hive.camera*, mountMeasured=true")
            put("2", "Limelight pipeline ${startup.tagPipelineIndex}: AprilTag 36h11, 82.55 mm, Full 3D")
            put("3", "Each CELL: spread < ~1 in, all four tags agree (else rotation convention)")
            put("4", "Tape-measure tripod → CELL opening centre; compare horizontal")
            put("5", "Tip each HIVE; note tag heights both ways; set hive.raisedMinHeightIn between")
            put("6", "A saves a lab record (after START); B clears")
        }
    }
}

/**
 * The survey's memory: the latest row of every season tag and the latest fused
 * goal of every CELL seen this run, so a lab record can list all 16 tags even
 * when they were surveyed from different spots. Times are frame capture times.
 */
internal class HiveSurveyMemory {

    data class Seen<T>(val value: T, val captureNanos: Long)

    private val tags = TreeMap<Int, Seen<TagRow>>()
    private val cells = LinkedHashMap<Cell, Seen<CellGoal>>()

    fun record(hive: HiveTracker, captureNanos: Long) {
        for (row in hive.rows) tags[row.tag.id] = Seen(row, captureNanos)
        for ((cell, goal) in hive.cellGoals) cells[cell] = Seen(goal, captureNanos)
    }

    fun tag(id: Int): Seen<TagRow>? = tags[id]

    fun cell(cell: Cell): Seen<CellGoal>? = cells[cell]

    fun reset() {
        tags.clear()
        cells.clear()
    }

    /** `[measured]` lines for a lab record: HIVE states, every CELL, every tag. */
    fun measurements(hive: HiveTracker, nowNanos: Long): List<Pair<String, String>> {
        val m = ArrayList<Pair<String, String>>()
        for (alliance in Alliance.entries) {
            val state = hive.state(alliance)?.name ?: "UNKNOWN"
            m += "hive.${alliance.name}" to "$state${if (hive.stateAssumed(alliance)) " (assumed)" else ""}; tips ${hive.tipCount(alliance)}"
        }
        for (cell in BiobuzzAprilTags.cells) {
            val seen = cells[cell]
            m += "cell.${cell.stickerLabel.replace(' ', '_')}" to
                (seen?.let { describeCell(it.value) + "; ${ago(nowNanos, it.captureNanos)}" } ?: "not seen")
        }
        for (tag in BiobuzzAprilTags.tags) {
            val seen = tags[tag.id]
            m += "tag.${tag.id}" to (seen?.let { describeTag(it.value) + "; ${ago(nowNanos, it.captureNanos)}" } ?: "${tag.meaning}; not seen")
        }
        return m
    }
}

internal fun describeCell(goal: CellGoal): String =
    "${goal.heightClass} goal ${vec(goal.goalRobot)} in; " +
        "horizontal ${fmt(GoalGeometry.horizontalDistanceFromTurretIn(goal.goalRobot), 1)} in; " +
        "turret bearing ${fmt(Math.toDegrees(GoalGeometry.turretBearingRad(goal.goalRobot)), 1)}°; " +
        "spread ${fmt(goal.spreadIn, 2)} in; tags ${goal.tagIds.joinToString(",")}"

internal fun describeTag(row: TagRow): String =
    "${row.tag.meaning}; ${row.heightClass} at ${fmt(row.tagRobot.z, 1)} in; goal ${vec(row.goalRobot)} in; " +
        "off fused ${fmt(row.deviationIn, 2)} in${if (row.usedInFusion) "" else " (dropped)"}; " +
        "faces ${fmt(row.facingBelowHorizontalDeg, 0)}° below horizontal"

private fun vec(v: Vec3) = "(${fmt(v.x, 1)}, ${fmt(v.y, 1)}, ${fmt(v.z, 1)})"

private fun ago(nowNanos: Long, thenNanos: Long) = "seen ${fmt((nowNanos - thenNanos) / 1e9, 1)} s ago"
