package org.firstinspires.ftc.teamcode.core.logging

import java.io.ByteArrayOutputStream
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/** Round-trip: everything [WpiLogWriter] emits, [WpiLog] reads back. */
class WpiLogReaderTest {

    @Test
    fun roundTripsEveryRecordType() {
        val buffer = ByteArrayOutputStream()
        val writer = WpiLogWriter(buffer, "test-header")
        val d = writer.startEntry("mech/position", "double")
        val i = writer.startEntry("loop/count", "int64")
        val s = writer.startEntry("events", "string")
        val b = writer.startEntry("flags/homed", "boolean")
        val a = writer.startEntry("pose", "double[]")

        writer.appendDouble(d, 1.5, 100L)
        writer.appendDouble(d, -2.25, 200L)
        writer.appendInt64(i, 42L, 100L)
        writer.appendInt64(i, -7L, 300L)
        writer.appendString(s, "auto step 1/5: follow", 150L)
        writer.appendBoolean(b, true, 400L)
        writer.appendDoubleArray(a, doubleArrayOf(1.0, 2.0, 3.5), 500L)
        writer.flush()

        val log = WpiLog.read(buffer.toByteArray())

        assertEquals("double", log.entries.values.first { it.name == "mech/position" }.type)
        assertEquals(listOf(100L to 1.5, 200L to -2.25), log.doubles("mech/position"))
        assertEquals(listOf(100L to 42L, 300L to -7L), log.longs("loop/count"))
        assertEquals(listOf(150L to "auto step 1/5: follow"), log.strings("events"))
        assertEquals(listOf(400L to true), log.booleans("flags/homed"))

        val arrays = log.doubleArrays("pose")
        assertEquals(1, arrays.size)
        assertEquals(500L, arrays[0].first)
        assertArrayEquals(doubleArrayOf(1.0, 2.0, 3.5), arrays[0].second, 0.0)

        assertTrue(log.has("pose"))
        assertTrue(!log.has("nonexistent"))
        assertTrue(log.doubles("nonexistent").isEmpty())
    }

    @Test
    fun timestampsSurviveLargeValues() {
        val buffer = ByteArrayOutputStream()
        val writer = WpiLogWriter(buffer)
        val d = writer.startEntry("x", "double")
        val bigTs = 123_456_789_012L // > 4 bytes worth of microseconds
        writer.appendDouble(d, 9.0, bigTs)
        writer.flush()

        assertEquals(listOf(bigTs to 9.0), WpiLog.read(buffer.toByteArray()).doubles("x"))
    }
}
