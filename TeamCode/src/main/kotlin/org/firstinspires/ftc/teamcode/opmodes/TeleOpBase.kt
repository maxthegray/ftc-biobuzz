package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

/**
 * Base for teleop op-modes. Registers the drive subsystem, switches the
 * follower into teleop mode at start, and provides the standard control
 * block every teleop wants — so season op-modes start from mechanisms,
 * not boilerplate.
 *
 * Subclass contract:
 *  - register season subsystems and trigger bindings in [configureTeleop]
 *  - call [standardDriveControls] from [onLoop] whenever the driver should
 *    be in manual control (skip it while a command owns the drive)
 *  - if you override [onStart], call `super.onStart()` so the follower
 *    still enters teleop mode
 */
abstract class TeleOpBase : OpModeBase() {

    protected lateinit var drive: MecanumDriveSubsystem
        private set

    /** Override to localise off something else (e.g. an SRSHub Pinpoint). */
    protected open fun createFollower(): Follower = Constants.createFollower(hardwareMap)

    final override fun configure() {
        drive = robot.register(MecanumDriveSubsystem(createFollower()))
        configureTeleop()
    }

    /** Register additional subsystems and wire trigger bindings here. */
    protected open fun configureTeleop() {}

    override fun onStart() {
        drive.enableTeleop()
    }

    /**
     * The standard drive-control block:
     *  - **Back + Y** — reset heading to zero (Back, not Start: Start+A/B
     *    are the Driver Station's gamepad re-bind chords)
     *  - **Back + B** — toggle field-centric / robot-centric
     *  - **right trigger** — precision mode while held
     *  - left stick translate, right stick turn (per [DriveConfig] scaling)
     *
     * Returns true while precision mode is active, for telemetry.
     */
    protected fun standardDriveControls(): Boolean {
        if (driver.back && driver.yPressed) {
            drive.setPose(drive.pose.withHeading(0.0))
        }
        if (driver.back && driver.bPressed) {
            DriveConfig.fieldCentric = !DriveConfig.fieldCentric
        }
        val precision = driver.rightTrigger > 0.1
        drive.drive(
            forward = driver.leftStickY,
            strafe = driver.leftStickX,
            turn = driver.rightStickX,
            precision = precision,
        )
        return precision
    }
}
