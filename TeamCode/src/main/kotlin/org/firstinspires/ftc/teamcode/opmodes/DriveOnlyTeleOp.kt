package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.Localizer

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
    private lateinit var localizer: Localizer

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(Localizer(follower))
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
        // Allow the driver to reset heading on the fly if odometry drifts.
        if (driver.start && driver.aPressed) {
            localizer.setPose(drive.pose.withHeading(0.0))
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
        val p = robot.profile
        telemetryBag.section("Loop (ms)") {
            put("total", p.totalNanos / 1e6, decimals = 2)
            put("clearCaches", p.clearCachesNanos / 1e6, decimals = 2)
            put("periodic", p.periodicNanos / 1e6, decimals = 2)
            put("control", p.controlNanos / 1e6, decimals = 2)
            put("scheduler", p.schedulerNanos / 1e6, decimals = 2)
            put("writeHardware", p.writeHardwareNanos / 1e6, decimals = 2)
            put("overhead", p.overheadNanos / 1e6, decimals = 2)
        }
        telemetryBag.section("Robot") {
            put("loopHz", robot.loopHz, decimals = 1)
            put("loopCount", robot.loopCount)
        }
    }
}

