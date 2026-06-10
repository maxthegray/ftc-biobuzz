package org.firstinspires.ftc.teamcode.core.util

/**
 * Monotonic nanosecond time source. Everything in the runtime that measures
 * durations ([org.firstinspires.ftc.teamcode.core.runtime.Robot],
 * [TelemetryBag], [org.firstinspires.ftc.teamcode.core.hardware.I2CBusThread])
 * reads time through this so JVM unit tests can substitute a controllable
 * fake instead of `System.nanoTime()`.
 */
fun interface Clock {
    fun nanos(): Long

    companion object {
        /** Production clock — delegates to [System.nanoTime]. */
        @JvmField
        val SYSTEM: Clock = Clock { System.nanoTime() }
    }
}
