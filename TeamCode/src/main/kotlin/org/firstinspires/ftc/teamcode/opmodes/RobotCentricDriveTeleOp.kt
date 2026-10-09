package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.DriveConfig

// Starting robot centric drive for greyson top-speed testing

@TeleOp(name = "Robot Centric Drive", group = "Match")
class RobotCentricDriveTeleOp : TeleOpBase() {

    override val turnScale = 0.7

    override fun configureTeleop() {
        if (drive.fieldCentric) drive.toggleFieldCentric()
    }

    override fun onLoop() {
        val precision = driver.r2 > 0.1
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.driveModeName)
            put("fieldCentric", drive.fieldCentric)
            put("inputExponent", DriveConfig.inputExponent)
            put("precision", precision)
            put("turnScale", turnScale)
        }
    }
}
