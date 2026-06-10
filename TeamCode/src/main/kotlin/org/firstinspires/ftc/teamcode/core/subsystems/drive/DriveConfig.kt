package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.bylazar.configurables.annotations.Configurable
import dev.frozenmilk.sinister.loading.Pinned

/**
 * Runtime drive tuning knobs.
 *
 * Physical constants (mass, forward/lateral zero-power acceleration, motor
 * names, Pinpoint pod offsets, etc.) live in
 * [org.firstinspires.ftc.teamcode.pedroPathing.Constants] — that is the file
 * Pedro reads when it builds the Follower. DriveConfig holds the other
 * things: driver feel, teleop scaling, and default brake behavior.
 *
 * These are `var`s so Panels can mutate them live during tuning sessions.
 *
 * [Pinned] so Sloth loads this object exactly once and never re-runs its
 * static initialiser on hot reload. Without it, every reload would reset
 * these fields to the defaults below and wipe whatever Panels tuned live.
 * Trade-off: edits to *this file* need a full install, not a hot reload.
 */
@Pinned
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

    /** If true, teleop uses field-centric translation (heading from the localizer). */
    @JvmField var fieldCentric: Boolean = true

    /**
     * When true the follower enters teleop with brake mode engaged: motors
     * actively hold when commanded zero. Set false if you want zero-power
     * coasting for smoother driver feel.
     */
    @JvmField var brakeOnTeleop: Boolean = true

    /** Inches-per-second below which [MecanumDriveSubsystem.isMoving] reports false. */
    @JvmField var stoppedVelocityThreshold: Double = DEFAULT_STOPPED_VELOCITY_THRESHOLD

    /** Default tolerances used when holding a pose at the end of an auton path. */
    @JvmField var holdToleranceInches: Double = DEFAULT_HOLD_TOLERANCE_INCHES

    /** Default heading tolerance (radians) for pose holds. */
    @JvmField var holdToleranceRadians: Double = DEFAULT_HOLD_TOLERANCE_RADIANS

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
