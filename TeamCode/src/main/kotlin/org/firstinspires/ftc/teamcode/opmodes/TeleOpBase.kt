package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.RumbleAlert
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.DriveConfig
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.TeleopInput

/**
 * Base for teleop op-modes. Registers the drive + localizer subsystems,
 * installs the stick-driven default command, and wires the standard driver
 * chords, so season op-modes start from mechanisms, not boilerplate.
 *
 * Standard controls:
 *  - left stick translate, right stick turn (per [DriveConfig] scaling)
 *  - **R2** — precision mode while held
 *  - **Create + Triangle** — reset heading to zero (Create, not Options:
 *    Options+Cross/Circle are the Driver Station's gamepad re-bind chords)
 *  - **Create + Circle** — toggle field-centric / robot-centric
 *
 * Localization starts at (0, 0, 0) every run; nothing is restored from a
 * previous op-mode (see [LocalizerSubsystem]).
 *
 * On a localizer fault the drive switches, for the rest of the run, to
 * robot-centric sticks that bypass odometry; paths, holds and turns refuse.
 *
 * Subclass contract: register season subsystems and trigger bindings in
 * [configureTeleop]. The drive default resumes after any other
 * drive command ends; there is nothing to call from [onLoop].
 */
abstract class TeleOpBase : OpModeBase() {

    override val rumbleAlerts: List<RumbleAlert> = listOf(
        RumbleAlert(RobotConfig.Match.FLOWER_UNLOCK_NANOS, blips = 2, expiresNanos = RobotConfig.Match.DRIVER_NANOS),
        RumbleAlert(RobotConfig.Match.FINAL_TWENTY_NANOS, blips = 3, expiresNanos = RobotConfig.Match.DRIVER_NANOS),
    )

    /** Multiplier on the turn stick; lower it for gentler rotation. */
    protected open val turnScale = 1.0

    protected lateinit var drive: MecanumDriveSubsystem
        private set

    protected lateinit var localizer: LocalizerSubsystem
        private set

    final override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        // Drive first, localizer second: the localizer samples pose history
        // right after the drive's writeHardware() runs Follower.update().
        drive = robot.register(MecanumDriveSubsystem(follower))
        val stickInput = {
            TeleopInput(
                forward = driver.leftStickY,
                strafe = driver.leftStickX,
                turn = driver.rightStickX * turnScale,
                precision = driver.r2 > 0.1,
            )
        }
        drive.defaultCommand = drive.teleopCommand(name = "Driver sticks", input = stickInput)
        val faultFallback = drive.robotCentricFallbackCommand(name = "Driver sticks (robot-centric fallback)", input = stickInput)
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
        (driver.button(Button.CREATE) and driver.button(Button.TRIANGLE)).onTrue(
            // Claiming the drive preempts a path or assist: hard-snapping the
            // pose under a live path controller would command a large jerk.
            logged(
                "Reset heading",
                instant { localizer.setPose(drive.pose.withHeading(0.0)) }.requiring(drive),
            ),
        )
        (driver.button(Button.CREATE) and driver.button(Button.CIRCLE)).onTrue(
            instant { drive.toggleFieldCentric() },
        )
        configureTeleop()
    }

    /** Register additional subsystems and wire trigger bindings here. */
    protected open fun configureTeleop() {}
}
