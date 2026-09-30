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

/**
 * Flywheel speed for a shot at the HIVE goal. The input is
 * `GoalObservation.horizontalDistanceIn`: horizontal distance from the turret
 * axis to the raised CELL's opening centre, inches.
 *
 * Not yet calibrated: fit this from shots at measured distances. Until then it
 * returns null, and callers must not spin the flywheel from it.
 */
object ShotModel {
    fun rpmForDistance(distanceIn: Double): Double? = null
}
