package org.firstinspires.ftc.teamcode.core.logging

import java.io.Closeable
import java.io.Flushable
import java.io.OutputStream
import java.nio.charset.StandardCharsets

/**
 * Minimal WPILOG 1.0 writer.
 *
 * All integers are little-endian and records use the compact header-length
 * bitfield from the WPILib data log format.
 */
class WpiLogWriter(
    private val out: OutputStream,
    extraHeader: String = "",
) : Closeable, Flushable {
    private var nextEntryId = 1

    init {
        out.write("WPILOG".toByteArray(StandardCharsets.US_ASCII))
        writeUInt16(0x0100)
        writeStringBytes(extraHeader)
    }

    fun startEntry(
        name: String,
        type: String,
        metadata: String = "",
        timestampUs: Long = 0L,
    ): Int {
        val id = nextEntryId++
        writeControl(timestampUs) {
            writeByte(0)
            writeUInt32(id)
            writeStringBytes(name)
            writeStringBytes(type)
            writeStringBytes(metadata)
        }
        return id
    }

    fun finishEntry(entryId: Int, timestampUs: Long = 0L) {
        writeControl(timestampUs) {
            writeByte(1)
            writeUInt32(entryId)
        }
    }

    fun setMetadata(entryId: Int, metadata: String, timestampUs: Long = 0L) {
        writeControl(timestampUs) {
            writeByte(2)
            writeUInt32(entryId)
            writeStringBytes(metadata)
        }
    }

    fun appendDouble(entryId: Int, value: Double, timestampUs: Long) {
        writeRecord(entryId, timestampUs, 8) {
            writeUInt64(java.lang.Double.doubleToRawLongBits(value))
        }
    }

    fun appendDoubleArray(entryId: Int, values: DoubleArray, timestampUs: Long) {
        writeRecord(entryId, timestampUs, values.size * 8) {
            for (value in values) writeUInt64(java.lang.Double.doubleToRawLongBits(value))
        }
    }

    fun appendInt64(entryId: Int, value: Long, timestampUs: Long) {
        writeRecord(entryId, timestampUs, 8) {
            writeUInt64(value)
        }
    }

    fun appendString(entryId: Int, value: String, timestampUs: Long) {
        val bytes = value.toByteArray(StandardCharsets.UTF_8)
        writeRecord(entryId, timestampUs, bytes.size) {
            out.write(bytes)
        }
    }

    fun appendBoolean(entryId: Int, value: Boolean, timestampUs: Long) {
        writeRecord(entryId, timestampUs, 1) {
            writeByte(if (value) 1 else 0)
        }
    }

    override fun flush() {
        out.flush()
    }

    override fun close() {
        out.close()
    }

    private inline fun writeControl(timestampUs: Long, payload: PayloadWriter.() -> Unit) {
        val buffer = java.io.ByteArrayOutputStream()
        PayloadWriter(buffer).payload()
        writeRecord(0, timestampUs, buffer.size()) {
            buffer.writeTo(out)
        }
    }

    private inline fun writeRecord(
        entryId: Int,
        timestampUs: Long,
        payloadSize: Int,
        payload: () -> Unit,
    ) {
        val entryLen = byteLength(entryId.toLong(), maxBytes = 4)
        val sizeLen = byteLength(payloadSize.toLong(), maxBytes = 4)
        val timeLen = byteLength(timestampUs, maxBytes = 8)
        val header = (entryLen - 1) or ((sizeLen - 1) shl 2) or ((timeLen - 1) shl 4)
        writeByte(header)
        writeLittle(entryId.toLong(), entryLen)
        writeLittle(payloadSize.toLong(), sizeLen)
        writeLittle(timestampUs, timeLen)
        payload()
    }

    private fun byteLength(value: Long, maxBytes: Int): Int {
        var length = 1
        var shifted = value ushr 8
        while (shifted != 0L && length < maxBytes) {
            length++
            shifted = shifted ushr 8
        }
        return length
    }

    private fun writeStringBytes(value: String) {
        val bytes = value.toByteArray(StandardCharsets.UTF_8)
        writeUInt32(bytes.size)
        out.write(bytes)
    }

    private fun writeByte(value: Int) {
        out.write(value and 0xff)
    }

    private fun writeUInt16(value: Int) = writeLittle(value.toLong(), 2)
    private fun writeUInt32(value: Int) = writeLittle(value.toLong(), 4)
    private fun writeUInt64(value: Long) = writeLittle(value, 8)

    private fun writeLittle(value: Long, bytes: Int) {
        for (i in 0 until bytes) out.write(((value ushr (8 * i)) and 0xff).toInt())
    }

    private class PayloadWriter(private val out: OutputStream) {
        fun writeStringBytes(value: String) {
            val bytes = value.toByteArray(StandardCharsets.UTF_8)
            writeUInt32(bytes.size)
            out.write(bytes)
        }

        fun writeByte(value: Int) {
            out.write(value and 0xff)
        }

        fun writeUInt32(value: Int) = writeLittle(value.toLong(), 4)

        private fun writeLittle(value: Long, bytes: Int) {
            for (i in 0 until bytes) out.write(((value ushr (8 * i)) and 0xff).toInt())
        }
    }
}
