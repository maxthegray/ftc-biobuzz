package org.firstinspires.ftc.teamcode.core.util

/**
 * Fixed-capacity ring buffer of timestamped doubles with no per-sample
 * allocation, for looking a measurement up at another sensor's capture time
 * (e.g. a turret encoder angle at a camera frame's exposure).
 *
 * Timestamps are monotonic nanoseconds from [Clock] and must not decrease.
 * [lookup] interpolates linearly and returns null outside the retained window.
 * Values are not wrapped: a mechanism angle that passes ±π stays continuous.
 */
class ScalarHistory(private val capacity: Int = 256) {
    init {
        require(capacity > 1) { "ScalarHistory capacity must be at least 2" }
    }

    private val times = LongArray(capacity)
    private val values = DoubleArray(capacity)
    private var next = 0
    private var size = 0

    fun add(timestampNanos: Long, value: Double) {
        times[next] = timestampNanos
        values[next] = value
        next = (next + 1) % capacity
        if (size < capacity) size++
    }

    fun newestTimestamp(): Long? = if (size == 0) null else times[physicalIndex(size - 1)]

    fun lookup(timestampNanos: Long): Double? {
        if (size == 0) return null
        val oldest = physicalIndex(0)
        val newest = physicalIndex(size - 1)
        if (timestampNanos < times[oldest] || timestampNanos > times[newest]) return null
        for (i in size - 1 downTo 1) {
            val b = physicalIndex(i)
            val a = physicalIndex(i - 1)
            if (timestampNanos > times[b] || timestampNanos < times[a]) continue
            if (times[b] == times[a]) return values[b]
            val u = (timestampNanos - times[a]).toDouble() / (times[b] - times[a]).toDouble()
            return values[a] + (values[b] - values[a]) * u
        }
        return values[oldest]
    }

    fun clear() {
        next = 0
        size = 0
    }

    private fun physicalIndex(logicalIndex: Int): Int =
        (next - size + logicalIndex + capacity) % capacity
}
