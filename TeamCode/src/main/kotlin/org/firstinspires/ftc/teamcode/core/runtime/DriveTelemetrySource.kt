package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.math.Velocity

/**
 * The drive state the flight recorder and the Panels field view consume, so
 * observability code and its tests don't need a real follower. Members must
 * be cheap and exception-free: they run on the hot loop.
 */
interface DriveTelemetrySource {
    /** Field pose: inches, radians, CCW-positive heading, Pedro field frame. */
    val pose: Pose

    /** Field-frame velocity: inches/second and radians/second. */
    val velocity: Velocity

    /** "IDLE", "TELEOP", "FOLLOWING", "HOLDING" or "ROBOT_CENTRIC_FALLBACK". */
    val driveModeName: String

    /** True while following a path or holding a pose (follow errors are live). */
    val isPathing: Boolean

    /** Distance from the closest path point (or hold target) in inches; NaN when unavailable. */
    val followTranslationalErrorInches: Double

    /** Signed heading error in radians; NaN when unavailable. */
    val followHeadingErrorRad: Double

    /** Points along the path being followed, for drawing; empty when not following. */
    fun currentPathPoints(samples: Int): List<Vector2D>
}
