package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig

/**
 *  Just driving teleop, copy and paste where needed
 */
@TeleOp(name = "Drive Only", group = "Match")
class DriveOnlyTeleOp : TeleOpBase() {

    override fun onLoop() {
        val precision = driver.rightTrigger > 0.1
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
            put("fieldCentric", drive.fieldCentric)
            put("inputExponent", DriveConfig.inputExponent)
            put("precision", precision)
        }
    }
}
