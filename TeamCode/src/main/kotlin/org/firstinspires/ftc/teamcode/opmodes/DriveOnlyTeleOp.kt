package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig

/**
 * Minimal teleop that exercises the starter scaffolding end-to-end:
 *
 *  - Drive subsystem + Pedro Follower registered by [TeleOpBase]
 *  - Field-centric mecanum drive with precision trigger
 *  - Back+Y resets the heading on the fly, handy when odometry drifts
 *  - Telemetry via [telemetryBag] (one call feeds both Driver Station + Panels)
 *
 * Wiring: copy this file, rename the class, register season subsystems in
 * `configureTeleop()`, and build up `onLoop` as needed.
 */
@TeleOp(name = "Starter: Drive Only", group = "Starter")
class DriveOnlyTeleOp : TeleOpBase() {

    override fun onLoop() {
        val precision = standardDriveControls()

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
