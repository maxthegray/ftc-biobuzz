package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.commands.Commands
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Base for teleop op-modes. Registers the drive + localizer subsystems,
 * installs the stick-driven teleop default command, and wires the standard
 * driver chords — so season op-modes start from mechanisms, not boilerplate.
 *
 * Standard controls:
 *  - left stick translate, right stick turn (per [DriveConfig] scaling)
 *  - **right trigger** — precision mode while held
 *  - **Back + Y** — reset heading to zero (Back, not Start: Start+A/B are
 *    the Driver Station's gamepad re-bind chords)
 *  - **Back + B** — toggle field-centric / robot-centric
 *
 * Subclass contract: register season subsystems and trigger bindings in
 * [configureTeleop]. The drive default command resumes automatically after
 * any higher-priority action ends; there is nothing to call from [onLoop].
 */
abstract class TeleOpBase : OpModeBase() {

    protected lateinit var drive: MecanumDriveSubsystem
        private set

    protected lateinit var localizer: LocalizerSubsystem
        private set

    /** Override to localise off something else (e.g. an SRSHub Pinpoint). */
    protected open fun createFollower(): Follower = Constants.createFollower(hardwareMap)

    /** Override to false for teleops that must not inherit auton's field pose. */
    protected open val restorePoseFromAuton: Boolean get() = true

    final override fun configure() {
        val follower = createFollower()
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
        (driver.button(Button.BACK) and driver.button(Button.Y))
            .onTrue(
                Commands.instant { localizer.setPose(drive.pose.withHeading(0.0)) }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
        (driver.button(Button.BACK) and driver.button(Button.B))
            .onTrue(
                Commands.instant { DriveConfig.fieldCentric = !DriveConfig.fieldCentric }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
        configureTeleop()
    }

    /** Register additional subsystems and wire trigger bindings here. */
    protected open fun configureTeleop() {}

    /** If you override this, call `super.onStart()` to keep the pose handoff. */
    override fun onStart() {
        if (restorePoseFromAuton) localizer.restorePersistedPose()
    }
}
