package org.firstinspires.ftc.teamcode.vision

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.abs
import kotlin.math.sign
import org.firstinspires.ftc.teamcode.core.control.PIDFController
import org.firstinspires.ftc.teamcode.core.control.PIDFGains

/**
 * Turns a Limelight vertical target error into a forward drive power, so the
 * robot closes on the ball. The companion to [BallAimController], which owns
 * heading — the two run together and tune independently.
 *
 * Control is on raw `ty`, not on a distance in inches: park the ball where the
 * robot should stop, read `ty`, and type it into
 * [BallApproachConfig.targetTyDegrees]. No camera calibration involved.
 *
 * A ball farther than the setpoint sits *higher* in frame, so `ty - targetTy` is
 * positive exactly when the robot needs to drive forward.
 */
class BallRangeController {

    private val gains = PIDFGains()
    private val controller = PIDFController(gains)

    /** True while `ty` is within [BallApproachConfig.deadbandDegrees] of the setpoint. */
    var atTarget: Boolean = false
        private set

    /** The ty the last [update] acted on, or `NaN` when there was no target. */
    var lastTyDegrees: Double = Double.NaN
        private set

    /** Forward power returned by the last [update]. */
    var lastOutput: Double = 0.0
        private set

    /** Clear controller state. Call whenever the approach is (re-)engaged. */
    fun reset() {
        controller.reset()
        atTarget = false
        lastTyDegrees = Double.NaN
        lastOutput = 0.0
    }

    /**
     * One control step. [tyDegrees] is null when no ball is visible, which stops
     * the robot dead. [txDegrees] gates it: driving forward while the ball is
     * well off to one side carves an arc around it instead of approaching, so
     * nothing moves until the aim assist has brought it within
     * [BallApproachConfig.alignGateDegrees].
     */
    fun update(dtSeconds: Double, tyDegrees: Double?, txDegrees: Double?): Double {
        gains.kP = BallApproachConfig.safeKp
        gains.kD = BallApproachConfig.safeKd

        if (tyDegrees == null || !tyDegrees.isFinite()) {
            reset()
            return 0.0
        }

        lastTyDegrees = tyDegrees

        val target = BallApproachConfig.safeTargetTyDegrees
        if (abs(tyDegrees - target) <= BallApproachConfig.safeDeadbandDegrees) {
            controller.reset()
            atTarget = true
            lastOutput = 0.0
            return 0.0
        }
        atTarget = false

        val aligned = txDegrees != null &&
            txDegrees.isFinite() &&
            abs(txDegrees) <= BallApproachConfig.safeAlignGateDegrees
        if (!aligned) {
            controller.reset()
            lastOutput = 0.0
            return 0.0
        }

        // Driving forward makes ty *fall*, so the loop is closed on -ty, which
        // rises as the robot advances and so gives the controller the positive
        // plant gain its sign convention assumes.
        val raw = controller.calculate(
            dtSeconds,
            measurement = -tyDegrees,
            targetPosition = -target,
        )
        val max = BallApproachConfig.safeMaxForwardPower
        var output = if (raw.isFinite()) raw.coerceIn(-max, max) else 0.0

        // Friction floor, but only on an output the controller actually asked
        // for: with kP left at its zero default the approach must stay still.
        val min = BallApproachConfig.safeMinForwardPower
        if (output != 0.0 && abs(output) < min) output = min * sign(output)

        lastOutput = output
        return output
    }

}

/**
 * Live-tunable gains for the Limelight ball approach.
 *
 * Same contract as [BallAimConfig]: `@JvmField var`s for Panels, `safe*`
 * accessors so a bad slider can't NaN the motors, and
 * [org.firstinspires.ftc.teamcode.core.runtime.ConfigStore] registration so
 * tuned values survive power cycles, installs, and hot reloads.
 *
 * **[kP] ships at zero on purpose.** Until a gain is typed in, the approach
 * binding behaves exactly like the plain aim assist — so this is safe to have
 * installed before anyone has decided the robot should drive itself at
 * anything.
 */
@Configurable
object BallApproachConfig {

    private const val DEFAULT_KP = 0.0
    private const val DEFAULT_KD = 0.0
    private const val DEFAULT_TARGET_TY_DEGREES = 0.0
    private const val DEFAULT_MAX_FORWARD_POWER = 0.30
    private const val DEFAULT_MIN_FORWARD_POWER = 0.08
    private const val DEFAULT_DEADBAND_DEGREES = 1.0
    private const val DEFAULT_ALIGN_GATE_DEGREES = 25.0

    /**
     * Forward power per degree of range error. Zero by default, which makes the
     * approach inert — raise it only after [targetTyDegrees] is measured.
     */
    @JvmField var kP: Double = DEFAULT_KP

    /** Damping on the rate of change of ty. Leave at 0 until kP is tuned. */
    @JvmField var kD: Double = DEFAULT_KD

    /**
     * The `ty` the robot settles at — the standoff. Park the ball where the
     * robot should stop, read `ty` off `Limelight Ball Test`, put it here.
     */
    @JvmField var targetTyDegrees: Double = DEFAULT_TARGET_TY_DEGREES

    /**
     * Hard cap on approach power. Deliberately low: the vision loop carries
     * 20–50 ms of latency, and a robot that outruns its own measurements
     * oscillates around the ball instead of settling on it.
     */
    @JvmField var maxForwardPower: Double = DEFAULT_MAX_FORWARD_POWER

    /** Static-friction floor, applied only outside the deadband. */
    @JvmField var minForwardPower: Double = DEFAULT_MIN_FORWARD_POWER

    /** Half-width of the "close enough" window, in degrees of ty error. */
    @JvmField var deadbandDegrees: Double = DEFAULT_DEADBAND_DEGREES

    /** Beyond this much horizontal error, don't drive forward at all — turn first. */
    @JvmField var alignGateDegrees: Double = DEFAULT_ALIGN_GATE_DEGREES

    internal val safeKp: Double get() = finiteAtLeast(kP, min = 0.0, fallback = 0.0)

    internal val safeKd: Double get() = finiteAtLeast(kD, min = 0.0, fallback = 0.0)

    internal val safeTargetTyDegrees: Double
        get() = if (targetTyDegrees.isFinite()) targetTyDegrees else DEFAULT_TARGET_TY_DEGREES

    internal val safeMaxForwardPower: Double
        get() = finiteAtLeast(maxForwardPower, min = 0.0, fallback = DEFAULT_MAX_FORWARD_POWER)
            .coerceAtMost(1.0)

    internal val safeMinForwardPower: Double
        get() = finiteAtLeast(minForwardPower, min = 0.0, fallback = 0.0)
            .coerceAtMost(safeMaxForwardPower)

    internal val safeDeadbandDegrees: Double
        get() = finiteAtLeast(deadbandDegrees, min = 0.0, fallback = DEFAULT_DEADBAND_DEGREES)

    internal val safeAlignGateDegrees: Double
        get() = finiteAtLeast(alignGateDegrees, min = 0.0, fallback = DEFAULT_ALIGN_GATE_DEGREES)

    private fun finiteAtLeast(value: Double, min: Double, fallback: Double): Double =
        if (value.isFinite() && value >= min) value else fallback
}
