package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

/**
 * Minimal teleop that exercises the starter scaffolding end-to-end:
 *
 *  - Subsystems registered on [robot]
 *  - Pedro Follower built from [Constants.createFollower]
 *  - Field-centric mecanum drive with precision trigger
 *  - Back+Y resets the heading on the fly, handy when odometry drifts
 *  - Telemetry via [telemetryBag] (one call feeds both Driver Station + Panels)
 *
 * Wiring: copy this file, rename the class, and build up subsystems as needed.
 */
@TeleOp(name = "Starter: Drive Only", group = "Starter")
class DriveOnlyTeleOp : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
        // Allow the driver to reset heading on the fly if odometry drifts.
        // Back+Y, not Start+A — Start+A is the Driver Station's gamepad
        // re-bind chord and would fire on a mid-match re-pair.
        if (driver.back && driver.yPressed) {
            drive.setPose(drive.pose.withHeading(0.0))
        }

        // Toggle field-centric vs robot-centric with back + B.
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

        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
            put("fieldCentric", DriveConfig.fieldCentric)
            put("inputExponent", DriveConfig.inputExponent)
            put("precision", precision)
        }
        // Loop-phase timing is already published by OpModeBase ("Loop").
        telemetryBag.section("Robot") {
            put("loopCount", robot.loopCount)
        }
    }
}

