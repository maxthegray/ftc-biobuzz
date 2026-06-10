package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import org.firstinspires.ftc.teamcode.core.util.shortestHeadingDelta

/**
 * Fixed-capacity pose ring buffer with no per-tick allocation.
 *
 * Timestamps are monotonic nanoseconds from [org.firstinspires.ftc.teamcode.core.util.Clock].
 * [lookup] linearly interpolates between bracketing samples and returns null
 * when the requested timestamp is outside the retained window.
 */
class PoseHistory(private val capacity: Int = 512) {
    init {
        require(capacity > 1) { "PoseHistory capacity must be at least 2" }
    }

    private val times = LongArray(capacity)
    private val xs = DoubleArray(capacity)
    private val ys = DoubleArray(capacity)
    private val headings = DoubleArray(capacity)

    private var next = 0
    private var size = 0

    fun add(timestampNanos: Long, pose: Pose) {
        times[next] = timestampNanos
        xs[next] = pose.x
        ys[next] = pose.y
        headings[next] = MathFunctions.normalizeAngle(pose.heading)
        next = (next + 1) % capacity
        if (size < capacity) size++
    }

    fun lookup(timestampNanos: Long): Pose? {
        if (size == 0) return null
        val oldest = physicalIndex(0)
        val newest = physicalIndex(size - 1)
        if (timestampNanos < times[oldest] || timestampNanos > times[newest]) return null
        if (timestampNanos == times[oldest]) return poseAt(oldest)
        if (timestampNanos == times[newest]) return poseAt(newest)

        for (i in 0 until size - 1) {
            val a = physicalIndex(i)
            val b = physicalIndex(i + 1)
            val ta = times[a]
            val tb = times[b]
            if (timestampNanos < ta || timestampNanos > tb) continue
            if (tb == ta) return poseAt(b)

            val u = (timestampNanos - ta).toDouble() / (tb - ta).toDouble()
            val headingDelta = shortestHeadingDelta(headings[a], headings[b])
            return Pose(
                xs[a] + (xs[b] - xs[a]) * u,
                ys[a] + (ys[b] - ys[a]) * u,
                MathFunctions.normalizeAngle(headings[a] + headingDelta * u),
            )
        }
        return null
    }

    private fun physicalIndex(logicalIndex: Int): Int =
        (next - size + logicalIndex + capacity) % capacity

    private fun poseAt(index: Int): Pose = Pose(xs[index], ys[index], headings[index])
}
