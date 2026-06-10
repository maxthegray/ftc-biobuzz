package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig

/**
 * Minimal teleop that exercises the starter scaffolding end-to-end:
 *
 *  - [TeleOpBase] registers drive + localizer, installs the teleop default
 *    command, and wires the Back+Y / Back+B driver chords
 *  - Pose handoff from auton via the persisted-pose restore in `onStart`
 *  - Telemetry via [telemetryBag] (one call feeds both Driver Station + Panels)
 *
 * Wiring: copy this file, rename the class, and build up subsystems in
 * [configureTeleop] as the season progresses.
 */
@TeleOp(name = "Starter: Drive Only", group = "Starter")
class DriveOnlyTeleOp : TeleOpBase() {

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
