package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.subsystems.turret.FixedTurretAngle
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.apriltags.TagSightingTracker
import org.firstinspires.ftc.teamcode.vision.hive.HiveGoalConfig
import org.firstinspires.ftc.teamcode.vision.hive.HiveGoalSubsystem
import org.firstinspires.ftc.teamcode.vision.hive.HiveSurvey
import org.firstinspires.ftc.teamcode.vision.hive.HiveSurvey.Companion.fmt
import org.firstinspires.ftc.teamcode.vision.hive.TurretCameraMountConfig
import org.firstinspires.ftc.teamcode.vision.logging.SettingsChangeLog

/**
 * Stationary survey of the HIVE tags with the Limelight on a tripod (turret
 * angle held at 0; `turretCamera` describes the tripod). For every tag of both
 * alliances it shows the tag's height, raised/lowered class, the CELL opening
 * centre it implies and how far that is from its CELL's fused goal, plus each
 * HIVE's inferred state. Four tags of one CELL agreeing within about an inch
 * checks the camera mount, FIRST's cluster geometry and the Limelight's
 * rotation convention together. Commands no motors.
 *
 * Driver A (after START) saves a lab record of everything seen this run;
 * B clears the survey memory.
 */
@TeleOp(name = "Hive Tag Survey", group = "Diagnostics")
class HiveTagSurveyTeleOp : OpModeBase() {

    private lateinit var startup: VisionDiagnostics.Startup
    private lateinit var limelight: LimelightSubsystem
    private lateinit var hive: HiveGoalSubsystem
    private lateinit var recorder: VisionLabRecorder
    private val survey = HiveSurvey()
    private val sightings = TagSightingTracker()

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(VisionDiagnostics.limelightRequirement)
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        ConfigStore.register("hiveGoal", HiveGoalConfig, HiveGoalConfig::resetDefaults)
        ConfigStore.register("turretCamera", TurretCameraMountConfig, TurretCameraMountConfig::resetDefaults)
        startup = VisionDiagnostics.registerConfigsAndLoad()
        robot.recordEvent(SettingsChangeLog.describe("hiveGoal", HiveGoalConfig))
        robot.recordEvent(SettingsChangeLog.describe("turretCamera", TurretCameraMountConfig))
        limelight = robot.register(VisionDiagnostics.limelight(startup))
        hive = robot.register(
            HiveGoalSubsystem(limelight, FixedTurretAngle(0.0), eventSink = robot::recordEvent, clock = robot.clock),
        )
        recorder = robot.register(
            VisionLabRecorder(
                "Hive Tag Survey",
                extraSections = listOf(
                    "hiveGoal" to HiveGoalConfig.compiledDefaults(),
                    "turretCamera" to TurretCameraMountConfig.compiledDefaults(),
                ),
            ),
        )
        driver.button(Button.A).onTrue(
            instant { recorder.save(survey.measurements(hive.tracker, hive.mount, robot.clock.nanos())) },
        )
        driver.button(Button.B).onTrue(
            instant {
                survey.reset()
                robot.recordEvent("Hive Tag Survey memory cleared")
            },
        )
    }

    override fun onStart() {
        robot.recordEvent("Hive Tag Survey started; mount measured=${TurretCameraMountConfig.measured}")
    }

    override fun onInitLoop() = tick()

    override fun onLoop() = tick()

    private fun tick() {
        val now = robot.clock.nanos()
        if (limelight.newFrameThisTick && limelight.resultFresh && limelight.pipelineMatches) {
            sightings.recordFrame(limelight.fiducials, now)
        }
        if (hive.newFrameThisTick) hive.lastFrameCaptureNanos?.let { survey.record(hive.tracker, it) }
        recorder.poll(robot)

        VisionDiagnostics.restartSection(telemetryBag, startup, null)
        VisionDiagnostics.limelightSections(telemetryBag, limelight, sightings, now, detailed = false)
        val mount = hive.mount
        val settings = hive.settings
        telemetryBag.section("Survey Setup") {
            put("mount", if (mount.measured) "measured" else "NOT MEASURED: heights and raised/lowered are meaningless")
            put(
                "lens",
                "height ${fmt(mount.heightIn, 1)} in, pitch up ${fmt(mount.pitchUpDeg, 1)}°, yaw ${fmt(mount.yawDeg, 1)}°, " +
                    "fwd ${fmt(mount.forwardIn, 1)} / left ${fmt(mount.leftIn, 1)} in from the turret axis",
            )
            put("turret", "held at 0° (tripod)")
            put("raised if tag above", "${fmt(settings.raisedMinHeightIn, 1)} ± ${fmt(settings.classificationMarginIn, 1)} in")
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
                put(
                    cell.stickerLabel,
                    seen?.let { HiveSurvey.describeCell(it.value, mount) + "; ${fmt((now - it.captureNanos) / 1e9, 1)} s ago" }
                        ?: "not seen",
                )
            }
        }
        telemetryBag.section("Tags (latest)") {
            for (tag in BiobuzzAprilTags.tags) {
                val seen = survey.tag(tag.id) ?: continue
                put("ID ${tag.id}", HiveSurvey.describeTag(seen.value) + "; ${fmt((now - seen.captureNanos) / 1e9, 1)} s ago")
            }
        }
        telemetryBag.section("Procedure") {
            put("1", "Tripod: measure lens height, pitch up; set turretCamera.*, measured=true")
            put("2", "Limelight pipeline ${startup.tagPipelineIndex}: AprilTag 36h11, 82.55 mm, Full 3D")
            put("3", "Each CELL: spread < ~1 in, all four tags agree (else rotation convention)")
            put("4", "Tape-measure turret axis → CELL opening centre; compare horizontal")
            put("5", "Tip each HIVE; note tag heights both ways; set hiveGoal.raisedMinHeightIn between")
            put("6", "A saves a lab record (after START); B clears")
        }
    }
}
