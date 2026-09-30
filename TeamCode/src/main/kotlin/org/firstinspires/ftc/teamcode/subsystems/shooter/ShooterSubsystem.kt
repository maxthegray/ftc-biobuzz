package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/** Flywheel motors; accepts target RPM independently of the future distance regression. */
class ShooterSubsystem : SubsystemBase("Shooter") {
    override fun init(hardwareMap: HardwareMap) {
        error("Shooter hardware is not configured yet")
    }

    override fun health(): String = "Hardware not configured"

    override fun logState(log: StateLog) {
        log.put("configured", false)
    }
}
