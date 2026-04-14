package org.firstinspires.ftc.teamcode.starter.drive

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
 */
object DriveConfig {

    /** Overall multiplier applied to every teleop motion input. */
    @JvmField var teleopPowerScale: Double = 1.0

    /** Multiplier applied while the precision-mode trigger is held. */
    @JvmField var precisionPowerScale: Double = 0.35

    /** If true, teleop uses field-centric translation (heading from the localizer). */
    @JvmField var fieldCentric: Boolean = true

    /**
     * When true the follower enters teleop with brake mode engaged: motors
     * coast to zero when commanded zero instead of shorting. Set false if
     * you need immediate stops (e.g. endgame balance).
     */
    @JvmField var brakeOnTeleop: Boolean = true

    /** Inches-per-second below which [MecanumDriveSubsystem.isMoving] reports false. */
    @JvmField var stoppedVelocityThreshold: Double = 0.5

    /** Default tolerances used when holding a pose at the end of an auton path. */
    @JvmField var holdToleranceInches: Double = 1.0

    /** Default heading tolerance (radians) for pose holds. */
    @JvmField var holdToleranceRadians: Double = Math.toRadians(2.0)
}
