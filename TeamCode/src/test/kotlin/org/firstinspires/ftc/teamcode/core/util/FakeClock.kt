package org.firstinspires.ftc.teamcode.core.util

/** Controllable [Clock] for tests. */
class FakeClock(start: Long = 1_000_000_000L) : Clock {
    var now: Long = start

    override fun nanos(): Long = now

    fun advanceMs(ms: Double) {
        now += (ms * 1_000_000.0).toLong()
    }

    fun advanceNanos(ns: Long) {
        now += ns
    }
}
