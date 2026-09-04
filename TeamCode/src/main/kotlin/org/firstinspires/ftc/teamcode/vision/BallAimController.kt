package org.firstinspires.ftc.teamcode.vision

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.abs
import kotlin.math.sign
import org.firstinspires.ftc.teamcode.core.control.PIDFController
import org.firstinspires.ftc.teamcode.core.control.PIDFGains
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightColorTarget

/**
 * Turns a Limelight horizontal target error into a drivetrain turn power, so
 * the robot faces the ball. Heading only — nothing here drives the robot
 * towards the target.
 *
 * Deliberately free of hardware types: doubles in, turn power out, so the whole
 * control law runs in host JUnit like [PIDFController] itself. The op-mode does
 * the hardware contact on both ends.
 *
 * Output sign is **CCW-positive**, matching Pedro's convention, and comes out
 * that way for free: Limelight `tx` is positive-right, and driving the error
 * `0 - tx` through the PID negates it.
 */
class BallAimController {

    private val gains = PIDFGains()
    private val controller = PIDFController(gains)

    /** True while the last [update] had a usable target. */
    var hasTarget: Boolean = false
        private set

    /** True while the target is within [BallAimConfig.deadbandDegrees] — aimed. */
    var onTarget: Boolean = false
        private set

    /** The tx the last [update] acted on, or `NaN` when there was no target. */
    var lastTxDegrees: Double = Double.NaN
        private set

    /** Turn power returned by the last [update]. */
    var lastOutput: Double = 0.0
        private set

    /** Clear controller state. Call whenever the assist is (re-)engaged. */
    fun reset() {
        controller.reset()
        hasTarget = false
        onTarget = false
        lastTxDegrees = Double.NaN
        lastOutput = 0.0
    }

    /**
     * Pick the tx to aim at: the largest color blob above
     * [BallAimConfig.minAreaPercent], so a second ball further away doesn't drag
     * the aim off the near one. Falls back to the Limelight's own primary target
     * when the per-target list is empty, and returns null when nothing qualifies.
     *
     * Both arguments come straight off `LimelightSubsystem`, which already
     * clears them when the result is stale, invalid, or from the wrong pipeline.
     */
    fun selectTx(
        targets: List<LimelightColorTarget>,
        primary: LimelightColorTarget?,
    ): Double? {
        val minArea = BallAimConfig.safeMinAreaPercent
        val best = targets.filter { it.areaPercent >= minArea }.maxByOrNull { it.areaPercent }
            ?: primary?.takeIf { it.areaPercent >= minArea }
        return best?.txDegrees?.takeIf { it.isFinite() }
    }

    /**
     * One control step. [txDegrees] is null when no ball is visible — the
     * controller then resets and commands zero, so a lost target stops the turn
     * dead rather than coasting on stale error.
     */
    fun update(dtSeconds: Double, txDegrees: Double?): Double {
        gains.kP = BallAimConfig.safeKp
        gains.kD = BallAimConfig.safeKd

        if (txDegrees == null || !txDegrees.isFinite()) {
            reset()
            return 0.0
        }

        hasTarget = true
        lastTxDegrees = txDegrees

        if (abs(txDegrees) <= BallAimConfig.safeDeadbandDegrees) {
            // Settled: drop the accumulated derivative state so re-acquiring
            // motion later doesn't kick off a stale error difference.
            controller.reset()
            onTarget = true
            lastOutput = 0.0
            return 0.0
        }
        onTarget = false

        val raw = controller.calculate(dtSeconds, measurement = txDegrees, targetPosition = 0.0)
        val max = BallAimConfig.safeMaxTurnPower
        var output = if (raw.isFinite()) raw.coerceIn(-max, max) else 0.0

        // Friction floor, but only on an output the controller actually asked
        // for: with kP left at zero the assist must stay still, not creep.
        val min = BallAimConfig.safeMinTurnPower
        if (output != 0.0 && abs(output) < min) output = min * sign(output)

        lastOutput = output
        return output
    }
}

/**
 * Live-tunable gains for the Limelight ball heading assist.
 *
 * Same contract as
 * [org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig]: public
 * `@JvmField var`s so Panels can mutate them mid-session, `safe*` accessors so a
 * fat-fingered slider can't feed a NaN into the motors, and registration with
 * [org.firstinspires.ftc.teamcode.core.runtime.ConfigStore] so tuned values
 * survive power cycles, full installs, and Sloth hot reloads.
 *
 * [BallAimController] re-reads these every tick, and [PIDFController] re-reads
 * its gains on every `calculate()`, so a slider change takes effect on the next
 * control loop with no op-mode restart.
 */
@Configurable
object BallAimConfig {

    private const val DEFAULT_KP = 0.020
    private const val DEFAULT_KD = 0.0
    private const val DEFAULT_MAX_TURN_POWER = 0.45
    private const val DEFAULT_MIN_TURN_POWER = 0.06
    private const val DEFAULT_DEADBAND_DEGREES = 1.5
    private const val DEFAULT_MIN_AREA_PERCENT = 0.15

    /** Turn power per degree of horizontal target error. The main tuning knob. */
    @JvmField var kP: Double = DEFAULT_KP

    /** Damping on the rate of change of tx. Leave at 0 until kP is tuned. */
    @JvmField var kD: Double = DEFAULT_KD

    /** Hard cap on assist turn power, so a far-off target can't spin the robot flat out. */
    @JvmField var maxTurnPower: Double = DEFAULT_MAX_TURN_POWER

    /**
     * Static-friction floor: outside the deadband a non-zero output is raised to
     * at least this magnitude, so the robot doesn't stall a degree short of the
     * target with a power too small to break stiction.
     */
    @JvmField var minTurnPower: Double = DEFAULT_MIN_TURN_POWER

    /** Half-width of the "close enough" window, in degrees of tx. Output is zero inside it. */
    @JvmField var deadbandDegrees: Double = DEFAULT_DEADBAND_DEGREES

    /** Color blobs smaller than this fraction of frame area are ignored as noise. */
    @JvmField var minAreaPercent: Double = DEFAULT_MIN_AREA_PERCENT

    internal val safeKp: Double get() = finiteAtLeast(kP, min = 0.0, fallback = 0.0)

    internal val safeKd: Double get() = finiteAtLeast(kD, min = 0.0, fallback = 0.0)

    internal val safeMaxTurnPower: Double
        get() = finiteAtLeast(maxTurnPower, min = 0.0, fallback = DEFAULT_MAX_TURN_POWER)
            .coerceAtMost(1.0)

    internal val safeMinTurnPower: Double
        get() = finiteAtLeast(minTurnPower, min = 0.0, fallback = 0.0)
            .coerceAtMost(safeMaxTurnPower)

    internal val safeDeadbandDegrees: Double
        get() = finiteAtLeast(deadbandDegrees, min = 0.0, fallback = DEFAULT_DEADBAND_DEGREES)

    internal val safeMinAreaPercent: Double
        get() = finiteAtLeast(minAreaPercent, min = 0.0, fallback = 0.0)

    private fun finiteAtLeast(value: Double, min: Double, fallback: Double): Double =
        if (value.isFinite() && value >= min) value else fallback
}
