package org.firstinspires.ftc.teamcode.general.pathing

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.general.core.Alliance
import org.firstinspires.ftc.teamcode.general.drive.MecanumDriveSubsystem

/**
 * Kotlin DSL over Pedro's [PathBuilder].
 *
 * Pedro's fluent API is fine but mixes chaining, vararg curves, and heading
 * interpolation in a way that's hard to read for multi-segment paths. This
 * DSL keeps the builder underneath — no new geometry, no wrapping types —
 * and adds:
 *
 *  - An explicit `start(Pose)` that seeds the first segment
 *  - Method names that match intent: [lineTo], [splineTo], [curveThrough]
 *  - Alliance-aware variants (`lineToMirrored`) for writing one path file
 *    per routine and running it on either side
 *  - Terminal [build] that returns a ready-to-follow [PathChain]
 *
 * Example:
 *
 * ```kotlin
 * val toScore = drive.path(Pose(9.0, 60.0, 0.0)) {
 *     lineTo(Pose(30.0, 60.0))
 *     splineTo(Pose(48.0, 40.0, Math.toRadians(-45.0)))
 *     constantHeading(Math.toRadians(-45.0))
 * }
 * drive.followPath(toScore)
 * ```
 */
class PathDSL internal constructor(
    private val follower: Follower,
    private val alliance: Alliance,
    startPose: Pose,
) {
    private val builder: PathBuilder = follower.pathBuilder()
    private var last: Pose = applyAlliance(startPose)

    /** Append a straight line from the last pose to [target]. */
    fun lineTo(target: Pose): PathDSL {
        val end = applyAlliance(target)
        builder.addPath(BezierLine(last, end))
        last = end
        return this
    }

    /**
     * Append a cubic Bézier spline from the last pose through [control] to
     * [end]. Use this for smooth curves where you care about the tangent.
     */
    fun splineTo(control: Pose, end: Pose): PathDSL {
        val c = applyAlliance(control)
        val e = applyAlliance(end)
        builder.addPath(BezierCurve(listOf(last, c, e)))
        last = e
        return this
    }

    /**
     * Shorthand spline that infers a single control point halfway along
     * the tangent from the last pose to [end]. Good-enough for gentle
     * curves; reach for [splineTo] when you need to hit a specific shape.
     */
    fun splineTo(end: Pose): PathDSL {
        val e = applyAlliance(end)
        val control = Pose((last.x + e.x) / 2.0, (last.y + e.y) / 2.0, e.heading)
        builder.addPath(BezierCurve(listOf(last, control, e)))
        last = e
        return this
    }

    /**
     * Append a Catmull-Rom curve that passes through [points]. [tension]
     * controls the tangent magnitude at each point; 0.5 is a balanced
     * default.
     */
    fun curveThrough(tension: Double, vararg points: Pose): PathDSL {
        val mirrored = Array(points.size) { applyAlliance(points[it]) }
        builder.curveThrough(tension, *mirrored)
        last = mirrored.last()
        return this
    }

    /**
     * Set linear heading interpolation on the most recent segment. Robot
     * will rotate smoothly from [startHeading] to [endHeading] along that
     * segment's parametric [0, 1]. Both headings are already in alliance
     * coordinates — they are not mirrored.
     */
    fun linearHeading(startHeading: Double, endHeading: Double): PathDSL {
        val sh = if (alliance == Alliance.BLUE) Math.PI - startHeading else startHeading
        val eh = if (alliance == Alliance.BLUE) Math.PI - endHeading else endHeading
        builder.setLinearHeadingInterpolation(sh, eh)
        return this
    }

    /** Lock heading to a constant value across the most recent segment. */
    fun constantHeading(heading: Double): PathDSL {
        val h = if (alliance == Alliance.BLUE) Math.PI - heading else heading
        builder.setConstantHeadingInterpolation(h)
        return this
    }

    /** Follow the path's tangent for heading (robot nose follows the curve). */
    fun tangentHeading(): PathDSL {
        builder.setTangentHeadingInterpolation()
        return this
    }

    /** Reverse-drive the most recent segment (motors run backward along it). */
    fun reversed(): PathDSL {
        builder.setReversed()
        return this
    }

    fun build(): PathChain = builder.build()

    private fun applyAlliance(pose: Pose): Pose = alliance.mirror(pose)
}

/**
 * Build a [PathChain] rooted at [startPose]. Call this from an op-mode to
 * get a fluent DSL; the returned chain can be handed to
 * [MecanumDriveSubsystem.followPath] or `PedroCommands.follow`.
 */
fun MecanumDriveSubsystem.path(
    startPose: Pose,
    alliance: Alliance = Alliance.RED,
    block: PathDSL.() -> Unit,
): PathChain {
    val dsl = PathDSL(follower, alliance, startPose)
    dsl.block()
    return dsl.build()
}
