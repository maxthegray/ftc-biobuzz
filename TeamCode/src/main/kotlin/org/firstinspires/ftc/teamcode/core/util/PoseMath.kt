package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import kotlin.math.abs

internal fun shortestHeadingDelta(from: Double, to: Double): Double {
    val delta = MathFunctions.normalizeAngleSigned(to - from)
    return if (abs(delta + Math.PI) < 1e-12) Math.PI else delta
}

internal fun rotateTranslation(x: Double, y: Double, theta: Double): Pair<Double, Double> {
    val rotated = Pose(x, y, 0.0).rotate(theta, false)
    return Pair(rotated.x, rotated.y)
}

internal fun relativePose(from: Pose, to: Pose): Pose {
    val (x, y) = rotateTranslation(to.x - from.x, to.y - from.y, -from.heading)
    return Pose(x, y, shortestHeadingDelta(from.heading, to.heading))
}

internal fun composePose(origin: Pose, delta: Pose): Pose {
    val (x, y) = rotateTranslation(delta.x, delta.y, origin.heading)
    return Pose(
        origin.x + x,
        origin.y + y,
        MathFunctions.normalizeAngle(origin.heading + delta.heading),
    )
}
