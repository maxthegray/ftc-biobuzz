package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.opmodes.TeleOpBase

/**
 * Throwaway teleop for the fault drills (OPERATIONS.md step 7): normal
 * driving, Square throws an Exception (command fault, op-mode keeps running),
 * L1 calls `TODO()` (op-mode ends). Neither command requires the drive.
 * Delete after the drill.
 */
@TeleOp(name = "Fault Drill", group = "Diagnostics")
class FaultDrillTeleOp : TeleOpBase() {

    override fun configureTeleop() {
        driver.button(Button.SQUARE).onTrue(
            logged("Fault drill: exception", Command.build().setExecute { error("fault drill") }),
        )
        driver.button(Button.L1).onTrue(
            logged("Fault drill: TODO", Command.build().setExecute { TODO("fault drill") }),
        )
    }

    override fun onInitLoop() = drillSection()

    override fun onLoop() = drillSection()

    private fun drillSection() {
        telemetryBag.section("Fault drill") {
            put("Square", "command fault (keeps running)")
            put("L1", "TODO() (ends op-mode)")
            put("mode", drive.driveModeName)
        }
    }
}
