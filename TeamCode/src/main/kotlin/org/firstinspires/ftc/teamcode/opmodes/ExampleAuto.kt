package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.pathing.autoRoutine
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.util.Alliance

/**
 * Minimal end-to-end auton: drive out 24", settle, "score" (a wait), turn,
 * and drive back. Not game code — it exists to exercise and demonstrate
 * every piece of the auton toolkit in one place:
 *
 *  - poses written once in RED coordinates, mirrored with [Alliance.mirror]
 *    (pose overload for waypoints, heading overload for the interpolation
 *    setters), so [ExampleAutoBlue] is just an alliance override
 *  - paths built with Pedro's native `PathBuilder`
 *  - sequencing via [autoRoutine] / [PedroAutoRunner]
 *  - the auton lifecycle: set the starting pose, never call
 *    `enableTeleop()`, schedule in [onStart], stop when the routine ends
 *
 * Copy this file as the skeleton for a real routine.
 */
@Autonomous(name = "Starter: Example Auto", group = "Starter")
open class ExampleAuto : OpModeBase() {

    // RED-coordinate poses. BLUE gets these mirrored automatically.
    private val startRed = Pose(8.0, 56.0, 0.0)
    private val outRed = Pose(32.0, 56.0, 0.0)

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var runner: PedroAutoRunner

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))

        val start = alliance.mirror(startRed)
        val out = alliance.mirror(outRed)
        drive.setStartingPose(start)

        val outPath = drive.follower.pathBuilder()
            .addPath(BezierLine(start, out))
            .setConstantHeadingInterpolation(alliance.mirror(0.0))
            .build()

        val backPath = drive.follower.pathBuilder()
            .addPath(BezierLine(out, start))
            .setLinearHeadingInterpolation(
                alliance.mirror(Math.toRadians(90.0)),
                alliance.mirror(0.0),
            )
            .build()

        runner = autoRoutine(drive) {
            follow(outPath)
            holdPose(out)                                  // settle on the target pose
            wait(300)                                      // stand-in for "score"
            turnTo(alliance.mirror(Math.toRadians(90.0)))
            followAndHold(backPath)
        }
    }

    override fun onInitLoop() {
        telemetryBag.section("Auto") {
            put("alliance", alliance.name)
            put("start", alliance.mirror(startRed))
        }
    }

    override fun onStart() {
        // Note: no enableTeleop() — autons leave the follower in its
        // post-init passive state, ready for followPath.
        runner.schedule()
    }

    override fun onLoop() {
        telemetryBag.section("Auto") {
            put("pose", drive.pose)
            put("mode", drive.mode.name)
            put("done", runner.isDone)
        }
        if (runner.isDone) requestOpModeStop()
    }
}

@Autonomous(name = "Starter: Example Auto (Blue)", group = "Starter")
class ExampleAutoBlue : ExampleAuto() {
    override val alliance: Alliance get() = Alliance.BLUE
}
