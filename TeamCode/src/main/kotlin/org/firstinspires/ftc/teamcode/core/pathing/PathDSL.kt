package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.math.MathFunctions
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

/**
 * Kotlin DSL over Pedro's [PathBuilder].
 *
 * Pedro's fluent API is fine but mixes chaining, vararg curves, and heading
 * interpolation in a way that's hard to read for multi-segment paths. This
 * DSL keeps the builder underneath — waypoints come in as framework
 * [Pose2d]s and convert to Pedro geometry at this boundary — and adds:
 *
 *  - An explicit start pose that seeds the first segment
 *  - Method names that match intent: [lineTo], [splineTo], [curveThrough]
 *  - Alliance-aware mirroring: write every pose in RED coordinates and pass
 *    the alliance; BLUE transforms waypoints *and* heading interpolation
 *  - Terminal [build] that returns a ready-to-follow [PathChain]
 *
 * Example:
 *
 * ```kotlin
 * val toScore = drive.path(Pose2d(9.0, 60.0, 0.0), alliance = Alliance.RED) {
 *     lineTo(Pose2d(30.0, 60.0))
 *     splineTo(Pose2d(40.0, 50.0), Pose2d(48.0, 40.0))
 *     constantHeading(Math.toRadians(-45.0))
 * }
 * ```
 */
class PathDSL internal constructor(
    pathBuilderFactory: () -> PathBuilder,
    private val alliance: Alliance,
    startPose: Pose2d,
) {
    private val builder: PathBuilder = pathBuilderFactory()
    private var last: Pose2d = applyAlliance(startPose)
    private var hasSegment = false

    /** Append a straight line from the last pose to [target]. */
    fun lineTo(target: Pose2d): PathDSL {
        val end = applyAlliance(target)
        builder.addPath(BezierLine(last.toPedro(), end.toPedro()))
        last = end
        hasSegment = true
        return this
    }

    /**
     * Append a cubic Bézier spline from the last pose through [control] to
     * [end]. Use this for smooth curves where you care about the tangent.
     */
    fun splineTo(control: Pose2d, end: Pose2d): PathDSL {
        val c = applyAlliance(control)
        val e = applyAlliance(end)
        builder.addPath(BezierCurve(listOf(last.toPedro(), c.toPedro(), e.toPedro())))
        last = e
        hasSegment = true
        return this
    }

    /**
     * Append a Catmull-Rom curve that passes through [points]. [tension]
     * controls the tangent magnitude at each point; 0.5 is a balanced
     * default. When this is the first segment, a virtual prior point is
     * synthesized by reflecting the first waypoint across the start pose so
     * the entry tangent is sensible.
     */
    fun curveThrough(tension: Double, vararg points: Pose2d): PathDSL {
        require(points.isNotEmpty()) { "curveThrough requires at least one point" }
        val mirrored = Array(points.size) { applyAlliance(points[it]).toPedro() }
        if (hasSegment) {
            builder.curveThrough(tension, *mirrored)
        } else {
            val lastPedro = last.toPedro()
            val prevPoint = lastPedro.minus(mirrored[0].minus(lastPedro))
            builder.curveThrough(prevPoint, lastPedro, tension, *mirrored)
        }
        last = applyAlliance(points.last())
        hasSegment = true
        return this
    }

    /**
     * Set linear heading interpolation on the most recent segment. Robot
     * will rotate smoothly from [startHeading] to [endHeading] along that
     * segment's parametric [0, 1]. Headings are written in RED coordinates;
     * BLUE paths transform them per the season's field symmetry.
     */
    fun linearHeading(startHeading: Double, endHeading: Double): PathDSL {
        requireSegment("linearHeading")
        val sh = applyAllianceHeading(startHeading)
        val eh = applyAllianceHeading(endHeading)
        builder.setLinearHeadingInterpolation(sh, eh)
        return this
    }

    /** Lock heading to a constant value across the most recent segment. */
    fun constantHeading(heading: Double): PathDSL {
        requireSegment("constantHeading")
        val h = applyAllianceHeading(heading)
        builder.setConstantHeadingInterpolation(h)
        return this
    }

    /** Follow the path's tangent for heading (robot nose follows the curve). */
    fun tangentHeading(): PathDSL {
        requireSegment("tangentHeading")
        builder.setTangentHeadingInterpolation()
        return this
    }

    /** Reverse-drive the most recent segment (motors run backward along it). */
    fun reversed(): PathDSL {
        requireSegment("reversed")
        builder.setReversed()
        return this
    }

    fun build(): PathChain {
        requireSegment("build")
        return builder.build()
    }

    private fun applyAlliance(pose: Pose2d): Pose2d = alliance.mirror(pose)

    private fun applyAllianceHeading(heading: Double): Double =
        if (alliance == Alliance.BLUE) {
            MathFunctions.normalizeAngle(alliance.mirror(heading))
        } else {
            heading
        }

    private fun requireSegment(operation: String) {
        require(hasSegment) { "$operation requires at least one path segment" }
    }
}

/**
 * Build a [PathChain] rooted at [startPose]. Call this from an op-mode to
 * get a fluent DSL; the returned chain can be handed to
 * `MecanumDriveSubsystem.followCommand` or the auto runner.
 */
fun MecanumDriveSubsystem.path(
    startPose: Pose2d,
    alliance: Alliance,
    block: PathDSL.() -> Unit,
): PathChain {
    val dsl = PathDSL(follower::pathBuilder, alliance, startPose)
    dsl.block()
    return dsl.build()
}
