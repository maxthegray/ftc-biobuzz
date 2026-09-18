package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.vision.ball.BallCameraSubsystem
import org.firstinspires.ftc.teamcode.vision.logging.SettingsChangeLog
import org.firstinspires.ftc.teamcode.vision.apriltags.TagSightingTracker
import org.firstinspires.ftc.teamcode.vision.diagnostics.VisionDiagnosticsConfig

/**
 * Stationary Limelight 3A AprilTag diagnostic. Reports every fiducial in each
 * fresh result with its BIOBUZZ meaning, camera-relative measurements, and
 * freshness, plus a per-tag sighting history. Commands no motors, applies no
 * localization corrections, and infers nothing about HIVE state.
 *
 * Pipeline slot: `visionDiagnostics.limelightTagPipelineIndex` (default 1).
 * `visionDiagnostics.runBothCameras` also opens the USB ball camera to compare
 * loop timing under both loads. Driver A (after START) saves a lab record.
 */
@TeleOp(name = "Limelight AprilTag Test", group = "Diagnostics")
class LimelightAprilTagTestTeleOp : OpModeBase() {

    private lateinit var startup: VisionDiagnostics.Startup
    private lateinit var limelight: LimelightSubsystem
    private lateinit var recorder: VisionLabRecorder
    private var ballCamera: BallCameraSubsystem? = null
    private val sightings = TagSightingTracker()
    private val timing = LoopTimingStats()

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(VisionDiagnostics.limelightRequirement)
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        startup = VisionDiagnostics.registerConfigsAndLoad()
        robot.recordEvent(SettingsChangeLog.describe("visionDiagnostics", VisionDiagnosticsConfig))
        limelight = robot.register(VisionDiagnostics.limelight(startup))
        if (startup.runBothCameras) {
            Preflight.check(hardwareMap, listOf(VisionDiagnostics.ballCameraRequirement))
            ballCamera = robot.register(BallCameraSubsystem(eventSink = robot::recordEvent))
        }
        recorder = robot.register(VisionLabRecorder("Limelight AprilTag Test"))
        driver.button(Button.A).onTrue(
            instant {
                recorder.save(VisionDiagnostics.measurements(robot, timing, startup, limelight, sightings, ballCamera))
            },
        )
    }

    override fun onStart() {
        robot.recordEvent("Limelight AprilTag test started; both cameras=${startup.runBothCameras}")
    }

    override fun onInitLoop() = tick()

    override fun onLoop() {
        timing.record(robot.lastLoopNanos)
        tick()
    }

    private fun tick() {
        val now = robot.clock.nanos()
        if (limelight.newFrameThisTick && limelight.resultFresh && limelight.pipelineMatches) {
            sightings.recordFrame(limelight.fiducials, now)
        }
        recorder.poll(robot)
        VisionDiagnostics.restartSection(telemetryBag, startup, ballCamera)
        VisionDiagnostics.limelightSections(telemetryBag, limelight, sightings, now, detailed = true)
        ballCamera?.let { VisionDiagnostics.ballCameraSections(telemetryBag, it, detailed = false) }
        VisionDiagnostics.timingSection(telemetryBag, robot, timing, startup)
        telemetryBag.section("Procedure") {
            put("1", "Limelight web UI: pipeline ${startup.tagPipelineIndex} = AprilTag 36h11, 82.55 mm, Full 3D")
            put("2", "Hold one tag still; ID and meaning must match the sticker")
            put("3", "Move left/right: tx sign; up/down: ty sign; note both")
            put("4", "Tape-measure camera→tag; compare |d|")
            put("5", "A saves a lab record (after START); stop normally")
        }
    }
}
