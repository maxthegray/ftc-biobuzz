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
 * Stationary USB-camera ball diagnostic: yellow-pollen color/shape detection
 * through VisionPortal, live-tuned from Panels (`ballVision`), with a
 * Robot Controller preview of the original image, threshold mask, or detection
 * overlay. Commands no motors.
 *
 * Driver controls after START: A saves a lab record, X cycles the preview
 * mode, B toggles preview rendering. The camera and preview already run during
 * INIT. `visionDiagnostics.runBothCameras` also opens the Limelight.
 */
@TeleOp(name = "Ball Tracking Test", group = "Diagnostics")
class BallTrackingTestTeleOp : OpModeBase() {

    private lateinit var startup: VisionDiagnostics.Startup
    private lateinit var ballCamera: BallCameraSubsystem
    private lateinit var recorder: VisionLabRecorder
    private var limelight: LimelightSubsystem? = null
    private val sightings = TagSightingTracker()
    private val timing = LoopTimingStats()

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(VisionDiagnostics.ballCameraRequirement)
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        startup = VisionDiagnostics.registerConfigsAndLoad()
        robot.recordEvent(SettingsChangeLog.describe("visionDiagnostics", VisionDiagnosticsConfig))
        ballCamera = robot.register(BallCameraSubsystem(eventSink = robot::recordEvent))
        if (startup.runBothCameras) {
            Preflight.check(hardwareMap, listOf(VisionDiagnostics.limelightRequirement))
            limelight = robot.register(VisionDiagnostics.limelight(startup))
        }
        recorder = robot.register(VisionLabRecorder("Ball Tracking Test"))
        driver.button(Button.A).onTrue(
            instant {
                recorder.save(VisionDiagnostics.measurements(robot, timing, startup, limelight, sightings, ballCamera))
            },
        )
        driver.button(Button.X).onTrue(instant(VisionDiagnostics::cyclePreviewMode))
        driver.button(Button.B).onTrue(instant(VisionDiagnostics::togglePreview))
    }

    override fun onStart() {
        robot.recordEvent("Ball tracking test started; stream ${ballCamera.runningStream}; both cameras=${startup.runBothCameras}")
    }

    override fun onInitLoop() = tick()

    override fun onLoop() {
        timing.record(robot.lastLoopNanos)
        tick()
    }

    private fun tick() {
        val now = robot.clock.nanos()
        val ll = limelight
        if (ll != null && ll.newFrameThisTick && ll.resultFresh && ll.pipelineMatches) {
            sightings.recordFrame(ll.fiducials, now)
        }
        recorder.poll(robot)
        VisionDiagnostics.restartSection(telemetryBag, startup, ballCamera)
        VisionDiagnostics.ballCameraSections(telemetryBag, ballCamera, detailed = true)
        ll?.let { VisionDiagnostics.limelightSections(telemetryBag, it, sightings, now, detailed = false) }
        VisionDiagnostics.timingSection(telemetryBag, robot, timing, startup)
    }
}
