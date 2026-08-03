package org.firstinspires.ftc.teamcode.core.io

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * The hardware boundary for a single motor + encoder. Subsystems read and
 * write through this interface instead of holding a [DcMotorEx] directly, so
 * the same subsystem code runs against real hardware ([RealMotorIO]), a
 * physics stand-in ([SimMotorIO]) in host tests, or a recorded log (replay).
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
