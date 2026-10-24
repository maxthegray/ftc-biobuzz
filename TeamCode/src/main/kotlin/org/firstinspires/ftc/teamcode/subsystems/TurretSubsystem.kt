package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import org.firstinspires.ftc.teamcode.core.SubsystemBase

/**
 * Geared position servos; manual and camera aiming will share this hardware owner.
 *
 * The Limelight rides on the turret. Aim plan (see `vision/GoalAim`): with a
 * goal in view, drive its `tx` to zero; otherwise turn to
 * `GoalAim.pivotBearingRad(pose, alliance)`. Command angles through
 * [wrapToTravel] so the turret never winds past its travel.
 */
class TurretSubsystem : SubsystemBase("Turret") {
    val configured = false

    override fun init(hardwareMap: HardwareMap) {
        error("Turret hardware is not configured yet")
    }

    override fun health(): String = "Hardware not configured"

    companion object {
        /**
         * The angle to command for [target] (radians, CCW from the robot's
         * front): of every `target + 2πk` inside [min]..[max], the one nearest
         * [current]. Null when none is inside (the driver has to turn the robot).
         */
        fun wrapToTravel(target: Double, current: Double, min: Double, max: Double): Double? {
            val first = ceil((min - target) / (2 * PI)).toInt()
            val last = floor((max - target) / (2 * PI)).toInt()
            return (first..last).map { target + it * 2 * PI }.minByOrNull { abs(it - current) }
        }
    }
}
