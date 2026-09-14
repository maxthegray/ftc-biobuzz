package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Base for teleop op-modes. Registers the drive + localizer subsystems,
 * installs the stick-driven default command, and wires the standard driver
 * chords, so season op-modes start from mechanisms, not boilerplate.
 *
 * Standard controls:
 *  - left stick translate, right stick turn (per [DriveConfig] scaling)
 *  - **right trigger** — precision mode while held
 *  - **Back + Y** — reset heading to zero (Back, not Start: Start+A/B are
 *    the Driver Station's gamepad re-bind chords)
 *  - **Back + B** — toggle field-centric / robot-centric
 *
 * Localization starts at (0, 0, 0) every run; nothing is restored from a
 * previous op-mode (see [LocalizerSubsystem]).
 *
 * On a localizer fault the drive switches, for the rest of the run, to
 * robot-centric sticks that bypass odometry; paths, holds and turns refuse.
 *
 * Subclass contract: register season subsystems and trigger bindings in
 * [configureTeleop]. The drive default resumes after any higher-priority
 * drive command ends; there is nothing to call from [onLoop].
 */
abstract class TeleOpBase : OpModeBase() {

    protected lateinit var drive: MecanumDriveSubsystem
        private set

    protected lateinit var localizer: LocalizerSubsystem
        private set

    final override fun configure() {
        val follower = Constants.create(hardwareMap)
        // Drive first, localizer second: the localizer samples pose history
        // right after the drive's writeHardware() runs Follower.update().
        drive = robot.register(MecanumDriveSubsystem(follower))
        val stickInput = {
            TeleopInput(
                forward = driver.leftStickY,
                strafe = driver.leftStickX,
                turn = driver.rightStickX,
                precision = driver.rightTrigger > 0.1,
            )
        }
        drive.defaultCommand = drive.teleopCommand(input = stickInput)
        val faultFallback = drive.robotCentricFallbackCommand(stickInput)
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                onEvent = robot::recordEvent,
                isFollowing = drive::isFollowing,
                onFault = {
                    // Runs in periodic(), outside Scheduler.execute(): preempt the
                    // current drive owner now and keep assists from reclaiming it.
                    drive.defaultCommand = faultFallback
                    Scheduler.schedule(faultFallback)
                },
            ),
        )
        (driver.button(Button.BACK) and driver.button(Button.Y)).onTrue(
            // Claiming the drive preempts a path or assist: hard-snapping the
            // pose under a live path controller would command a large jerk.
            instant { localizer.setPose(drive.pose.withHeading(0.0)) }
                .requiring(drive)
                .setPriority(CommandPriorities.DRIVER_ACTION),
        )
        (driver.button(Button.BACK) and driver.button(Button.B)).onTrue(
            instant { drive.toggleFieldCentric() },
        )
        configureTeleop()
    }

    /** Register additional subsystems and wire trigger bindings here. */
    protected open fun configureTeleop() {}
}
