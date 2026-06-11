package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.pathing.autoRoutine
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.runtime.AutonSelector
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Minimal end-to-end auton: drive out 24", settle, "score" (a wait), turn,
 * and drive back. Not game code — it exists to exercise and demonstrate
 * every piece of the auton toolkit in one place:
 *
 *  - poses written once in RED coordinates; the `path` DSL mirrors waypoints
 *    *and* heading interpolation for BLUE, and [org.firstinspires.ftc.teamcode.core.util.Alliance.mirror]
 *    covers the bare `turnTo` heading
 *  - alliance / routine / start-delay picked on dpad in init via
 *    [AutonSelector] (press A to lock), so one op-mode serves both sides
 *  - sequencing via [autoRoutine] / [PedroAutoRunner]
 *  - the auton lifecycle: set the starting pose at start, never install a
 *    teleop default command, schedule in [onStart], stop when the routine
 *    ends — the final pose persists automatically for teleop to restore
 *
 * Copy this file as the skeleton for a real routine.
 */
@Autonomous(name = "Starter: Example Auto", group = "Starter")
class ExampleAuto : OpModeBase() {

    // RED-coordinate poses. BLUE gets these mirrored automatically.
    private val startRed = Pose2d(8.0, 56.0, 0.0)
    private val outRed = Pose2d(32.0, 56.0, 0.0)

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private lateinit var selector: AutonSelector
    private var runner: PedroAutoRunner? = null

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        // Drive first, localizer second — pose history is sampled after
        // Follower.update() in the drive's writeHardware.
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(LocalizerSubsystem(follower, onEvent = robot::recordEvent))
        selector = AutonSelector(robot, telemetryBag)
            .register("Out and back") { outAndBack() }
    }

    /** Built at start, after the selector has fixed the alliance. */
    private fun outAndBack(): PedroAutoRunner {
        val outPath = drive.path(startPose = startRed, alliance = alliance) {
            lineTo(outRed)
            constantHeading(0.0)
        }
        val backPath = drive.path(startPose = outRed, alliance = alliance) {
            lineTo(startRed)
            linearHeading(Math.toRadians(90.0), 0.0)
        }
        return autoRoutine(robot, drive, robot::recordEvent) {
            if (selector.startDelaySec > 0) wait(selector.startDelaySec * 1000L)
            follow(outPath)
            holdPose(alliance.mirror(outRed))              // settle on the target pose
            wait(300)                                      // stand-in for "score"
            turnTo(alliance.mirror(Math.toRadians(90.0)))
            followAndHold(backPath)
        }
    }

    override fun onInitLoop() {
        selector.update(driver)
    }

    override fun onStart() {
        localizer.setStartingPose(alliance.mirror(startRed))
        runner = selector.selectedRunner()?.schedule()
    }

    override fun onLoop() {
        telemetryBag.section("Auto") {
            put("alliance", alliance.name)
            put("pose", drive.pose)
            put("mode", drive.mode.name)
            put("done", runner?.isDone ?: false)
        }
        if (runner?.isDone == true) requestOpModeStop()
    }
}
