package org.firstinspires.ftc.teamcode.core.runtime

import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d

/**
 * Anything that knows where the robot is. Implemented by the drive and
 * localizer subsystems; consumed by logging and any season code that wants a
 * pose without coupling to a concrete subsystem class.
 */
interface PoseProvider {
    /** Field pose, inches + radians. */
    val pose: Pose2d

    /** Field-frame velocity in inches/second. */
    val velocity: Vector2d
}

/**
 * The drive-state surface the flight recorder and the Panels field view
 * consume. Lets observability code work against an interface instead of
 * reaching into `MecanumDriveSubsystem` (and through it into Pedro).
 *
 * Implementations must keep every member cheap and exception-free — these
 * are called on the hot loop's telemetry path.
 */
interface DriveTelemetrySource : PoseProvider {
    /** Short mode label, e.g. "TELEOP" / "FOLLOWING" / "HOLDING". */
    val driveModeName: String

    /** True while following a path or holding a pose (follow errors are live). */
    val isPathing: Boolean

    /** Angular velocity in rad/s; NaN if unavailable. */
    val angularVelocityRadPerSec: Double

    /** Translational follow error magnitude in inches; NaN when not pathing. */
    val followTranslationalErrorInches: Double

    /** Heading follow error in radians; NaN when not pathing. */
    val followHeadingErrorRad: Double

    /**
     * The active path(s) sampled for drawing: one list of poses per path
     * segment, [samplesPerPath]+1 poses each. Empty when nothing is being
     * followed.
     */
    fun currentPathPoses(samplesPerPath: Int): List<List<Pose2d>>
}
