package org.firstinspires.ftc.teamcode.core.control

import kotlin.math.abs
import kotlin.math.sign

/**
 * PIDF gains. Mutable fields so a season fork can hold them in a Panels
 * `@Configurable` object and tune live:
 *
 *  - [kP]/[kI]/[kD] — feedback on position error
 *  - [kS] — static-friction feedforward, signed by the setpoint velocity
 *  - [kV] — velocity feedforward (output per unit/s of setpoint velocity)
 *  - [kG] — constant gravity feedforward (lifts; for rotating arms wrap the
 *    controller and add a `cos(angle)` term at the call site)
 *  - [iMax] — cap on the magnitude of the *integral term's contribution*
 *    to the output, in output units
 */
class PIDFGains(
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kG: Double = 0.0,
    iMax: Double = Double.POSITIVE_INFINITY,
) {
    @JvmField var kP: Double = kP
    @JvmField var kI: Double = kI
    @JvmField var kD: Double = kD
    @JvmField var kS: Double = kS
    @JvmField var kV: Double = kV
    @JvmField var kG: Double = kG
    @JvmField var iMax: Double = iMax
}

/**
 * Position PID with kS/kV/kG feedforward, designed to be fed a profiled
 * setpoint (see [ProfiledController]). Gains are read every call, so live
 * tuning takes effect immediately. Single-threaded, like everything else in
 * the main loop.
 */
class PIDFController(val gains: PIDFGains) {

    private var integral = 0.0
    private var lastError = Double.NaN

    /** Clear integral and derivative state. Call when the mechanism jumps modes. */
    fun reset() {
        integral = 0.0
        lastError = Double.NaN
    }

    /**
     * One controller step. [targetVelocity] should be the profile setpoint's
     * velocity so kS/kV act as feedforward along the profile.
     */
    fun calculate(
        dtSeconds: Double,
        measurement: Double,
        targetPosition: Double,
        targetVelocity: Double = 0.0,
    ): Double {
        val error = targetPosition - measurement
        val derivative =
            if (lastError.isNaN() || dtSeconds <= 0.0) 0.0 else (error - lastError) / dtSeconds
        lastError = error

        var integralTerm = 0.0
        if (gains.kI != 0.0 && dtSeconds > 0.0) {
            integral += error * dtSeconds
            val cap = abs(gains.iMax / gains.kI)
            if (integral.isFinite() && cap.isFinite()) integral = integral.coerceIn(-cap, cap)
            integralTerm = gains.kI * integral
        }

        val feedforward = gains.kS * sign(targetVelocity) + gains.kV * targetVelocity + gains.kG
        return gains.kP * error + integralTerm + gains.kD * derivative + feedforward
    }
}
