package org.firstinspires.ftc.teamcode.core.logging

import java.io.ByteArrayOutputStream
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class WpiLogWriterTest {

    @Test
    fun headerMatchesWpiLogVersionOneWithEmptyExtraHeader() {
        val bytes = ByteArrayOutputStream()

        WpiLogWriter(bytes).flush()

        assertArrayEquals(
            byteArrayOf(
                0x57, 0x50, 0x49, 0x4c, 0x4f, 0x47,
                0x00, 0x01,
                0x00, 0x00, 0x00, 0x00,
            ),
            bytes.toByteArray(),
        )
    }

    @Test
    fun startControlRecordMatchesSpecFixture() {
        val bytes = ByteArrayOutputStream()
        val writer = WpiLogWriter(bytes)

        val id = writer.startEntry("test", "int64", timestampUs = 1_000_000L)

        assertEquals(1, id)
        assertArrayEquals(
            hex(
                "57 50 49 4c 4f 47 00 01 00 00 00 00 " +
                    "20 00 1a 40 42 0f " +
                    "00 01 00 00 00 " +
                    "04 00 00 00 74 65 73 74 " +
                    "05 00 00 00 69 6e 74 36 34 " +
                    "00 00 00 00",
            ),
            bytes.toByteArray(),
        )
    }

    @Test
    fun int64DataRecordMatchesSpecFixture() {
        val bytes = ByteArrayOutputStream()
        val writer = WpiLogWriter(bytes)

        writer.appendInt64(1, 3L, timestampUs = 1_000_000L)

        assertArrayEquals(
            hex("57 50 49 4c 4f 47 00 01 00 00 00 00 20 01 08 40 42 0f 03 00 00 00 00 00 00 00"),
            bytes.toByteArray(),
        )
    }

    private fun hex(value: String): ByteArray =
        value.trim().split(Regex("\\s+")).map { it.toInt(16).toByte() }.toByteArray()
}
