package org.firstinspires.ftc.teamcode.core.geometry

import java.util.Locale
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Framework-owned planar geometry. Everything outside the Pedro adapter
 * layer (`core/pathing/PedroConversions.kt`, the drive subsystem, the Pedro
 * `Localizer` implementations) speaks these types, so a Pedro version bump
 * churns one boundary instead of the whole codebase.
 *
 * Conventions: inches, radians, CCW-positive heading, field frame per Pedro
 * coordinates (origin at the RED-side corner). Headings are normalized to
 * [0, 2π) by every operation that produces one.
 */

/** Normalize an angle to [0, 2π). */
fun normalizeAngle(radians: Double): Double {
    var angle = radians % (2.0 * PI)
    if (angle < 0.0) angle += 2.0 * PI
    return angle
}

/** Normalize an angle to (-π, π]. */
fun normalizeAngleSigned(radians: Double): Double {
    val angle = normalizeAngle(radians)
    return if (angle > PI) angle - 2.0 * PI else angle
}

/**
 * Shortest signed rotation taking [from] to [to], in (-π, π]. An exact
 * half-turn resolves to +π so a correction never flips direction on noise.
 */
fun shortestAngleDelta(from: Double, to: Double): Double {
    val delta = normalizeAngleSigned(to - from)
    return if (abs(delta + PI) < 1e-12) PI else delta
}

/** Rotate the vector ([x], [y]) by [radians] CCW. */
internal fun rotateVector(x: Double, y: Double, radians: Double): Vector2d {
    val c = cos(radians)
    val s = sin(radians)
    return Vector2d(x * c - y * s, x * s + y * c)
}

/** A field-frame 2D vector (inches). */
data class Vector2d(
    @JvmField val x: Double,
    @JvmField val y: Double,
) {
    val magnitude: Double get() = hypot(x, y)

    operator fun plus(other: Vector2d): Vector2d = Vector2d(x + other.x, y + other.y)
    operator fun minus(other: Vector2d): Vector2d = Vector2d(x - other.x, y - other.y)
    operator fun times(scalar: Double): Vector2d = Vector2d(x * scalar, y * scalar)

    fun rotateBy(radians: Double): Vector2d = rotateVector(x, y, radians)

    override fun toString(): String = "(%.2f, %.2f)".format(Locale.US, x, y)

    companion object {
        @JvmField val ZERO = Vector2d(0.0, 0.0)
    }
}

/** A field pose: position in inches + heading in radians (CCW-positive). */
data class Pose2d(
    @JvmField val x: Double,
    @JvmField val y: Double,
    @JvmField val heading: Double = 0.0,
) {
    val translation: Vector2d get() = Vector2d(x, y)

    fun withHeading(heading: Double): Pose2d = Pose2d(x, y, heading)

    /** Straight-line distance to [other], ignoring heading. */
    fun distanceTo(other: Pose2d): Double = hypot(other.x - x, other.y - y)

    /**
     * This pose expressed in [origin]'s frame: the translation rotated into
     * origin's axes, the heading as the shortest delta from origin's.
     */
    fun relativeTo(origin: Pose2d): Pose2d {
        val rotated = rotateVector(x - origin.x, y - origin.y, -origin.heading)
        return Pose2d(rotated.x, rotated.y, shortestAngleDelta(origin.heading, heading))
    }

    /**
     * Apply [delta] (a pose expressed in this pose's frame, e.g. from
     * [relativeTo]) on top of this pose. Inverse of [relativeTo]:
     * `b == a.transformBy(b.relativeTo(a))`.
     */
    fun transformBy(delta: Pose2d): Pose2d {
        val rotated = rotateVector(delta.x, delta.y, heading)
        return Pose2d(
            x + rotated.x,
            y + rotated.y,
            normalizeAngle(heading + delta.heading),
        )
    }

    /** Linear interpolation toward [other]; heading takes the shortest arc. */
    fun lerp(other: Pose2d, t: Double): Pose2d = Pose2d(
        x + (other.x - x) * t,
        y + (other.y - y) * t,
        normalizeAngle(heading + shortestAngleDelta(heading, other.heading) * t),
    )

    /**
     * Map this RED-coordinate pose onto the BLUE side of a field of
     * [fieldLength] inches under the given [symmetry]. MIRROR matches Pedro's
     * `Pose.mirror(length)` exactly; ROTATE assumes a square field.
     */
    fun mirror(symmetry: FieldSymmetry, fieldLength: Double): Pose2d = when (symmetry) {
        FieldSymmetry.MIRROR -> Pose2d(fieldLength - x, y, normalizeAngle(PI - heading))
        FieldSymmetry.ROTATE -> Pose2d(fieldLength - x, fieldLength - y, normalizeAngle(heading + PI))
    }

    override fun toString(): String =
        "(%.2f, %.2f, %.1f°)".format(Locale.US, x, y, Math.toDegrees(heading))

    companion object {
        @JvmField val ZERO = Pose2d(0.0, 0.0, 0.0)
    }
}

/**
 * How the season's field maps RED coordinates onto BLUE. FTC alternates
 * between the two from game to game, so this is per-season configuration
 * ([org.firstinspires.ftc.teamcode.core.runtime.RobotConfig.Field.SYMMETRY])
 * — using MIRROR in a rotationally symmetric season silently produces wrong
 * BLUE paths.
 */
enum class FieldSymmetry {
    /** Reflection across the field's vertical centerline: (L−x, y, π−h). */
    MIRROR,

    /** 180° rotation about the field center: (L−x, L−y, h+π). Assumes a square field. */
    ROTATE,
}
