package org.firstinspires.ftc.teamcode.subsystems.turret

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/** Geared position servos; manual and camera aiming will share this hardware owner. */
class TurretSubsystem : SubsystemBase("Turret") {
    override fun init(hardwareMap: HardwareMap) {
        error("Turret hardware is not configured yet")
    }

    override fun periodic() {
        // Read hardware feedback here once the mechanism is defined.
    }

    override fun writeHardware() {
        // Apply command targets here once hardware is configured.
    }

    override fun onCommandFault() {
        // Clear targets and halt configured actuators.
    }

    override fun stop() {
        // Halt configured actuators immediately.
    }

    override fun health(): String = "Hardware not configured"

    override fun logState(log: StateLog) {
        log.put("configured", false)
    }
}
