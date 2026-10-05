package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo

/**
 * A motor and its encoder. Subsystems hold this instead of a [DcMotorEx] so
 * unit tests can swap in a simulated motor ([RealMotorIO] is the real one).
 *
 * Contract: reads are cheap (bulk-cache backed on real hardware) and happen
 * in `periodic()`; [setPower] is the only output and happens in
 * `writeHardware()`.
 */
interface MotorIO {
    /** Encoder position in ticks. */
    val positionTicks: Double

    /** Encoder velocity in ticks/second. */
    val velocityTicksPerSec: Double

    /** Commanded output power, [-1, 1]. */
    fun setPower(power: Double)

    /** Last power passed to [setPower]; for logging. */
    val lastPower: Double

    /** Zero the encoder at the current physical position. */
    fun resetEncoder()
}

/** [MotorIO] over a real [DcMotorEx]. Reads hit the Lynx bulk cache. */
class RealMotorIO(private val motor: DcMotorEx) : MotorIO {
    override val positionTicks: Double get() = motor.currentPosition.toDouble()
    override val velocityTicksPerSec: Double get() = motor.velocity

    override var lastPower: Double = 0.0
        private set

    override fun setPower(power: Double) {
        lastPower = power
        motor.power = power
    }

    override fun resetEncoder() {
        // Mode flip is the SDK's only encoder-zero mechanism; restore the
        // previous run mode so closed-loop code keeps its expectations.
        val mode = motor.mode
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
        } else {
            mode
        }
    }
}

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
