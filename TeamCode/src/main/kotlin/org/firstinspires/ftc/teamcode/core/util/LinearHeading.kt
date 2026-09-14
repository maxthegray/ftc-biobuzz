package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.math.Pose
import com.pedropathing.paths.interpolator.Interpolator
import com.pedropathing.utils.Angle

/**
 * Linear heading interpolation that is correct on every Pedro path:
 * `Paths.line(a, b).heading(linearHeading(a, b))`.
 *
 * Pedro 3.0.0's `Interpolator.linear` scales the turn by
 * `Curve.pathCompletion(t)`. The default implementation, used by `Line` and by
 * the curve of a compound `Paths.path(...)`, returns the fraction *remaining*,
 * so `.linear(a, b)` there starts at b's heading and ends at a's. Only
 * `BezierCurve` overrides it correctly. `remainingDistance(t)` is correct on all
 * three, so this uses the distance travelled instead. Like Pedro's, it turns the
 * short way and returns headings in [0, 2π). Pass poses already mapped through
 * [Alliance.poses] and it follows the mirrored headings.
 */
fun linearHeading(start: Double, end: Double): Interpolator {
    val from = Angle.normalize(start)
    val delta = Angle.error(from, Angle.normalize(end))
    return Interpolator { curve, t ->
        val length = curve.length()
        val travelled = if (length > 0.0) (1.0 - curve.remainingDistance(t) / length).coerceIn(0.0, 1.0) else 0.0
        Angle.normalize(from + delta * travelled)
    }
}

fun linearHeading(start: Pose, end: Pose): Interpolator = linearHeading(start.heading(), end.heading())
