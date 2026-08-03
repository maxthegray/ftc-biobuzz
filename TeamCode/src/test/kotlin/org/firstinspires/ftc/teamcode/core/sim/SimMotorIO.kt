package org.firstinspires.ftc.teamcode.core.sim

import kotlin.math.exp
import org.firstinspires.ftc.teamcode.core.io.MotorIO
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * First-order motor model for host tests: commanded power drives the
 * velocity toward `power × freeSpeedTicksPerSec` with time constant
 * [timeConstantSec]; position integrates the velocity. State advances
 * lazily from [clock] on every read, so tests just move a
 * [org.firstinspires.ftc.teamcode.core.sim.FakeClock] between ticks.
 *
 * Optional [minPositionTicks]/[maxPositionTicks] model hard stops: the
 * mechanism stalls (velocity 0) at the limit instead of passing through,
 * which is what homing routines detect.
 */
class SimMotorIO(
    private val clock: Clock,
    private val freeSpeedTicksPerSec: Double = 2800.0,
    private val timeConstantSec: Double = 0.1,
    private val minPositionTicks: Double = Double.NEGATIVE_INFINITY,
    private val maxPositionTicks: Double = Double.POSITIVE_INFINITY,
) : MotorIO {
    private var position = 0.0
    private var velocity = 0.0
    private var power = 0.0
    private var lastUpdateNs = clock.nanos()

    override val positionTicks: Double
        get() {
            advance()
            return position
        }

    override val velocityTicksPerSec: Double
        get() {
            advance()
            return velocity
        }

    override var lastPower: Double = 0.0
        private set

    override fun setPower(power: Double) {
        advance()
        lastPower = power
        this.power = power.coerceIn(-1.0, 1.0)
    }

    override fun resetEncoder() {
        advance()
        position = 0.0
    }

    /** Teleport the model (e.g. seed a start position in a test). */
    fun setPositionTicks(ticks: Double) {
        advance()
        position = ticks
        velocity = 0.0
    }

    private fun advance() {
        val now = clock.nanos()
        val dt = (now - lastUpdateNs) / 1e9
        lastUpdateNs = now
        if (dt <= 0.0) return

        val target = power * freeSpeedTicksPerSec
        val alpha = 1.0 - exp(-dt / timeConstantSec)
        val newVelocity = velocity + (target - velocity) * alpha
        position += (velocity + newVelocity) * 0.5 * dt
        velocity = newVelocity

        if (position <= minPositionTicks) {
            position = minPositionTicks
            if (velocity < 0.0) velocity = 0.0
        }
        if (position >= maxPositionTicks) {
            position = maxPositionTicks
            if (velocity > 0.0) velocity = 0.0
        }
    }
}
