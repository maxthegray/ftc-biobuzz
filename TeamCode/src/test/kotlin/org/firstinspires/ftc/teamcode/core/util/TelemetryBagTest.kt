package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.geometry.Pose
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TelemetryBagTest {

    private class FakeSink : TelemetryBag.Sink {
        val lines = mutableListOf<String>()
        val data = mutableListOf<Pair<String, String>>()
        var updates = 0

        override fun addLine(text: String) { lines += text }
        override fun addData(key: String, value: String) { data += key to value }
        override fun update() { updates++ }

        fun reset() {
            lines.clear(); data.clear(); updates = 0
        }
    }

    private val clock = FakeClock()
    private val sink = FakeSink()
    private val bag = TelemetryBag(listOf(sink), transmitIntervalMs = 50.0, clock = clock)

    @Test
    fun firstFlushAlwaysTransmits() {
        bag.section("S") { put("k", "v") }
        assertTrue(bag.flush())
        assertEquals(listOf("== S =="), sink.lines)
        assertEquals(listOf("k" to "v"), sink.data)
        assertEquals(1, sink.updates)
    }

    @Test
    fun flushThrottlesUntilIntervalElapses() {
        assertTrue(bag.flush())
        clock.advanceMs(10.0)
        assertFalse(bag.flush())
        clock.advanceMs(39.0)
        assertFalse(bag.flush())
        clock.advanceMs(1.0)
        assertTrue(bag.flush())
    }

    @Test
    fun sectionEntriesOverwriteWithinWindow() {
        bag.flush()
        sink.reset()
        bag.section("S") { put("k", "old") }
        bag.section("S") { put("k", "new") }
        clock.advanceMs(50.0)
        assertTrue(bag.flush())
        assertEquals(listOf("k" to "new"), sink.data)
    }

    @Test
    fun linesAccumulateAcrossThrottledTicks() {
        bag.flush()
        sink.reset()
        bag.line("first")
        clock.advanceMs(10.0)
        assertFalse(bag.flush())
        bag.line("second")
        clock.advanceMs(40.0)
        assertTrue(bag.flush())
        assertEquals(listOf("first", "second"), sink.lines)
    }

    @Test
    fun stateIsClearedAfterTransmit() {
        bag.section("S") { put("k", "v") }
        bag.line("event")
        bag.flush()
        sink.reset()
        clock.advanceMs(50.0)
        assertTrue(bag.flush())
        assertTrue(sink.lines.isEmpty())
        assertTrue(sink.data.isEmpty())
        assertEquals(1, sink.updates)
    }

    @Test
    fun doublesUseDecimalsHint() {
        bag.section("S") {
            put("a", 1.23456, decimals = 2)
            put("b", 2.0)
        }
        bag.flush()
        assertEquals(listOf("a" to "1.23", "b" to "2.000"), sink.data)
    }

    @Test
    fun doubleViaAnyOverloadStillFormats() {
        val anyValue: Any = 3.14159
        bag.section("S") { put("k", anyValue) }
        bag.flush()
        assertEquals(listOf("k" to "3.142"), sink.data)
    }

    @Test
    fun formatValueCases() {
        assertEquals("null", TelemetryBag.formatValue(null))
        assertEquals("0.500", TelemetryBag.formatValue(0.5))
        assertEquals("0.250", TelemetryBag.formatValue(0.25f))
        assertEquals("(1.00, 2.00, 90.0°)", TelemetryBag.formatValue(Pose(1.0, 2.0, Math.PI / 2)))
        assertEquals("text", TelemetryBag.formatValue("text"))
        assertEquals("true", TelemetryBag.formatValue(true))
    }

    @Test
    fun emptySectionsAreNotTransmitted() {
        bag.section("Empty") { }
        bag.flush()
        assertTrue(sink.lines.isEmpty())
    }
}
