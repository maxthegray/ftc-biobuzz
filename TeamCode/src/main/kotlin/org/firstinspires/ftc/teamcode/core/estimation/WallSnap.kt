package org.firstinspires.ftc.teamcode.core.estimation

import kotlin.math.PI
import kotlin.math.round
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngle
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig

/**
 * Builds the measured pose for a wall-contact relocalization: when a flat
 * face of the robot is pressed against a field wall, the axis perpendicular
 * to that wall and the heading are known exactly; the axis parallel to the
 * wall is not.
 *
 * Usage (e.g. on a driver chord after backing into the wall):
 *
 * ```kotlin
 * val measured = WallSnap.pose(
 *     wall = WallSnap.Wall.X_MIN,
 *     contactOffsetInches = 8.5,     // robot center → contact face
 *     current = localizer.pose,
 * )
 * localizer.applyCorrection(measured, robot.clock.nanos(), blend = 1.0)
 * ```
 *
 * The returned pose copies the current parallel-axis coordinate, so the
 * correction along that axis is zero — only the perpendicular axis and
 * heading actually move. (For belt-and-braces you can also pass reduced
 * weights, but copying makes it unnecessary.)
 */
object WallSnap {

    /** Field walls in Pedro coordinates: x and y each span [0, fieldLength]. */
    enum class Wall { X_MIN, X_MAX, Y_MIN, Y_MAX }

    /**
     * The pose implied by pressing the robot's face against [wall], with the
     * center [contactOffsetInches] from the contact face.
     *
     * [snapHeading] (default true) rounds the heading to the nearest
     * cardinal — flat contact means the face is parallel to the wall, so the
     * heading is a multiple of 90°. Pass false to keep the current heading
     * (e.g. contact via rollers that don't constrain rotation).
     */
    fun pose(
        wall: Wall,
        contactOffsetInches: Double,
        current: Pose2d,
        fieldLength: Double = RobotConfig.Field.LENGTH_INCHES,
        snapHeading: Boolean = true,
    ): Pose2d {
        require(contactOffsetInches >= 0.0) { "contactOffsetInches must be non-negative" }
        val heading = if (snapHeading) nearestCardinal(current.heading) else current.heading
        return when (wall) {
            Wall.X_MIN -> Pose2d(contactOffsetInches, current.y, heading)
            Wall.X_MAX -> Pose2d(fieldLength - contactOffsetInches, current.y, heading)
            Wall.Y_MIN -> Pose2d(current.x, contactOffsetInches, heading)
            Wall.Y_MAX -> Pose2d(current.x, fieldLength - contactOffsetInches, heading)
        }
    }

    /** Nearest multiple of 90°, normalized to [0, 2π). */
    fun nearestCardinal(headingRadians: Double): Double =
        normalizeAngle(round(normalizeAngle(headingRadians) / (PI / 2.0)) * (PI / 2.0))
}
