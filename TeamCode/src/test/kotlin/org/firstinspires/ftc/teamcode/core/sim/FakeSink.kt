package org.firstinspires.ftc.teamcode.core.sim

import org.firstinspires.ftc.teamcode.core.util.TelemetryBag

/**
 * Records everything a [TelemetryBag] transmits, so tests can assert on the
 * published output without a Driver Station or Panels.
 *
 * [data] keeps every `addData` call in order, including repeats, because the
 * bag's throttling and overwrite semantics are themselves under test. Use
 * [latest] when you only care about the value a key ended up with.
 */
class FakeSink : TelemetryBag.Sink {
    val lines = mutableListOf<String>()
    val data = mutableListOf<Pair<String, String>>()
    var updates = 0

    override fun addLine(text: String) {
        lines += text
    }

    override fun addData(key: String, value: String) {
        data += key to value
    }

    override fun update() {
        updates++
    }

    /** The most recently transmitted value for [key], or null if never sent. */
    fun latest(key: String): String? = data.lastOrNull { it.first == key }?.second

    fun reset() {
        lines.clear()
        data.clear()
        updates = 0
    }
}
