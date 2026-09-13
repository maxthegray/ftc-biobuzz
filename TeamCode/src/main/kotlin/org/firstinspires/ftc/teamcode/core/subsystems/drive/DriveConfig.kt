package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.bylazar.configurables.annotations.Configurable

/**
 * Runtime drive tuning knobs: driver feel, teleop scaling, brake behaviour and
 * the arrival tolerances drive commands measure against.
 *
 * Physical and Pedro constants (motor names and directions, Pinpoint offsets,
 * Foresight tuning) live in [org.firstinspires.ftc.teamcode.pedro.Constants].
 *
 * These are `var`s so Panels can mutate them live. Tuned values are persisted
 * by [org.firstinspires.ftc.teamcode.core.runtime.ConfigStore] and restored at
 * every op-mode init, so they survive power cycles, full installs, and Sloth
 * hot reloads without `@Pinned`.
 */
@Configurable
object DriveConfig {

    private const val DEFAULT_INPUT_EXPONENT = 2.0
    private const val DEFAULT_TELEOP_POWER_SCALE = 1.0
    private const val DEFAULT_PRECISION_POWER_SCALE = 0.35
    private const val DEFAULT_STOPPED_VELOCITY_THRESHOLD = 0.5
    private const val DEFAULT_HOLD_TOLERANCE_INCHES = 1.0
    private val DEFAULT_HOLD_TOLERANCE_RADIANS = Math.toRadians(2.0)

    /**
     * Exponent for the stick input curve applied to forward, strafe, and turn.
     * 1.0 = linear, 2.0 = squared (smooth at low speed), 3.0 = cubic.
     * Sign is always preserved so the robot still drives in the correct direction.
     * Mutate live via Panels / FTC Dashboard.
     */
    @JvmField var inputExponent: Double = DEFAULT_INPUT_EXPONENT

    /** Overall multiplier applied to every teleop motion input. */
    @JvmField var teleopPowerScale: Double = DEFAULT_TELEOP_POWER_SCALE

    /** Multiplier applied while the precision-mode trigger is held. */
    @JvmField var precisionPowerScale: Double = DEFAULT_PRECISION_POWER_SCALE

    /** Field-centric state copied into each drive subsystem at op-mode init. */
    @JvmField var fieldCentricDefault: Boolean = true

    /**
     * When true, manual driving uses brake mode: motors actively hold when
     * commanded zero. Applied to Pedro's `MecanumConfig.manualBrakeMode` when
     * the follower is created, so a change takes effect at the next op-mode init.
     */
    @JvmField var brakeOnTeleop: Boolean = true

    /** Inches-per-second below which [MecanumDriveSubsystem.isMoving] reports false. */
    @JvmField var stoppedVelocityThreshold: Double = DEFAULT_STOPPED_VELOCITY_THRESHOLD

    /** Default tolerances used when holding a pose at the end of an auton path. */
    @JvmField var holdToleranceInches: Double = DEFAULT_HOLD_TOLERANCE_INCHES

    /** Default heading tolerance (radians) for pose holds. */
    @JvmField var holdToleranceRadians: Double = DEFAULT_HOLD_TOLERANCE_RADIANS

    fun resetDefaults() {
        inputExponent = DEFAULT_INPUT_EXPONENT
        teleopPowerScale = DEFAULT_TELEOP_POWER_SCALE
        precisionPowerScale = DEFAULT_PRECISION_POWER_SCALE
        fieldCentricDefault = true
        brakeOnTeleop = true
        stoppedVelocityThreshold = DEFAULT_STOPPED_VELOCITY_THRESHOLD
        holdToleranceInches = DEFAULT_HOLD_TOLERANCE_INCHES
        holdToleranceRadians = DEFAULT_HOLD_TOLERANCE_RADIANS
    }

    internal val safeInputExponent: Double
        get() = finiteAtLeast(inputExponent, min = Double.MIN_VALUE, fallback = 1.0)

    internal val safeTeleopPowerScale: Double
        get() = finiteAtLeast(teleopPowerScale, min = 0.0, fallback = 0.0)

    internal val safePrecisionPowerScale: Double
        get() = finiteAtLeast(precisionPowerScale, min = 0.0, fallback = 0.0)

    internal val safeStoppedVelocityThreshold: Double
        get() = finiteAtLeast(
            stoppedVelocityThreshold,
            min = 0.0,
            fallback = DEFAULT_STOPPED_VELOCITY_THRESHOLD,
        )

    internal val safeHoldToleranceInches: Double
        get() = finiteAtLeast(holdToleranceInches, min = 0.0, fallback = DEFAULT_HOLD_TOLERANCE_INCHES)

    internal val safeHoldToleranceRadians: Double
        get() = finiteAtLeast(
            holdToleranceRadians,
            min = 0.0,
            fallback = DEFAULT_HOLD_TOLERANCE_RADIANS,
        )

    private fun finiteAtLeast(value: Double, min: Double, fallback: Double): Double =
        if (value.isFinite() && value >= min) value else fallback
}
