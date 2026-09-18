package org.firstinspires.ftc.teamcode.subsystems.intake

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/** Intake rollers; collect and reverse commands will be added with the hardware. */
class IntakeSubsystem : SubsystemBase("Intake") {
    override fun init(hardwareMap: HardwareMap) {
        error("Intake hardware is not configured yet")
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
