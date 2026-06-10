package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.commands.Commands
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Minimal teleop that exercises the starter scaffolding end-to-end:
 *
 *  - Subsystems registered on [robot]
 *  - Pedro Follower built from [Constants.createFollower]
 *  - Field-centric mecanum drive with precision trigger
 *  - Reset pose on `start` for a quick "zero the heading" handy for drivers
 *  - Telemetry via [telemetryBag] (one call feeds both Driver Station + Panels)
 *
 * Wiring: copy this file, rename the class, and build up subsystems as needed.
 */
@TeleOp(name = "Starter: Drive Only", group = "Starter")
class DriveOnlyTeleOp : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(LocalizerSubsystem(follower))
        drive.defaultCommand = drive.teleopCommand {
            TeleopInput(
                forward = driver.leftStickY,
                strafe = driver.leftStickX,
                turn = driver.rightStickX,
                precision = driver.rightTrigger > 0.1,
            )
        }
        (driver.button(Button.START) and driver.button(Button.A))
            .onTrue(
                Commands.instant { localizer.setPose(drive.pose.withHeading(0.0)) }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
        (driver.button(Button.BACK) and driver.button(Button.B))
            .onTrue(
                Commands.instant { DriveConfig.fieldCentric = !DriveConfig.fieldCentric }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
    }

    override fun onStart() {
        localizer.restorePersistedPose()
    }

    override fun onLoop() {
        val precision = driver.rightTrigger > 0.1
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
            put("fieldCentric", DriveConfig.fieldCentric)
            put("inputExponent", DriveConfig.inputExponent)
            put("precision", precision)
        }
    }
}
