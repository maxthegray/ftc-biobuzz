package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.pathing.autoRoutine
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.StartDelay
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Minimal end-to-end auton: drive out 24", settle, "score" (a wait), turn,
 * and drive back. Not game code — it exists to exercise and demonstrate
 * every piece of the auton toolkit in one place:
 *
 *  - poses written once in RED coordinates; the `path` DSL mirrors waypoints
 *    *and* heading interpolation for BLUE, and [org.firstinspires.ftc.teamcode.core.util.Alliance.mirror]
 *    covers the bare `turnTo` heading
 *  - one op-mode per alliance and routine, so the Driver Station dropdown is
 *    the selector: copy this file and override [initialAlliance] for BLUE.
 *    Only the start delay is picked at init, on dpad, via [StartDelay]
 *  - sequencing via [autoRoutine] / [PedroAutoRunner]
 *  - progress markers plus per-step and whole-routine timeouts
 *  - the auton lifecycle: abort on a pre-start localizer fault, set the
 *    starting pose at start, never install a teleop default command, require
 *    scheduling to succeed in [onStart], stop when the routine ends — the
 *    final pose persists automatically for teleop to restore
 *
 * Copy this file as the skeleton for a real routine.
 */
@Autonomous(name = "Example Auto", group = "Match")
class ExampleAuto : OpModeBase() {

    // RED-coordinate poses. BLUE gets these mirrored automatically.
    private val startRed = Pose2d(8.0, 56.0, 0.0)
    private val outRed = Pose2d(32.0, 56.0, 0.0)

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private lateinit var startDelay: StartDelay
    private var runner: PedroAutoRunner? = null

    /** The BLUE copy of this file overrides this and changes nothing else. */
    override val initialAlliance: Alliance get() = Alliance.RED

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        // Drive first, localizer second — pose history is sampled after
        // Follower.update() in the drive's writeHardware.
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                onEvent = robot::recordEvent,
                isFollowing = drive::isFollowing,
                // Watchdog policy for auton: driving blind is worse than
                // parking — cancel the routine and stop where we are.
                onFault = {
                    runner?.cancel()
                    drive.breakPath()
                },
            ),
        )
        startDelay = StartDelay(telemetryBag)
    }

    /** Built at start, so the paths see the final tuned config. */
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
            if (startDelay.millis > 0) wait(startDelay.millis)
            timeout(4_000) {
                follow(outPath) {
                    at(0.5, "outbound midpoint") {}
                }
            }
            holdPose(alliance.mirror(outRed))              // settle on the target pose
            wait(300)                                      // stand-in for "score"
            turnTo(alliance.mirror(Math.toRadians(90.0)))
            followAndHold(backPath)
        }.timeout(29_000)
    }

    override fun onInitLoop() {
        startDelay.update(driver)
    }

    override fun onStart() {
        localizer.fault?.let {
            abortAuto("localizer fault before start: $it")
            return
        }
        localizer.setStartingPose(alliance.mirror(startRed))
        val selected = outAndBack()
        runner = selected
        if (!selected.schedule()) abortAuto("routine schedule rejected")
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

    private fun abortAuto(reason: String) {
        robot.recordEvent("AUTO ABORTED: $reason")
        requestOpModeStop()
    }
}
