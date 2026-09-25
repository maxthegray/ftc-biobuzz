package org.firstinspires.ftc.teamcode.core.io

import com.qualcomm.robotcore.hardware.Servo

/**
 * The hardware boundary for a position servo, like [MotorIO] for motors.
 * [setPosition] is the only output and happens in `writeHardware()`.
 */
interface ServoIO {
    /** Commanded position, [0, 1]. */
    fun setPosition(position: Double)

    /** Last position passed to [setPosition], or NaN before the first; for logging. */
    val lastPosition: Double
}

/** [ServoIO] over a real [Servo]. Writes only when the position changes; the servo holds it. */
class RealServoIO(private val servo: Servo) : ServoIO {
    override var lastPosition: Double = Double.NaN
        private set

    override fun setPosition(position: Double) {
        if (position == lastPosition) return
        lastPosition = position
        servo.position = position
    }
}
