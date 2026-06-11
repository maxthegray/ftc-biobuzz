package org.firstinspires.ftc.teamcode.core.logging

import java.io.File
import java.nio.charset.StandardCharsets

/**
 * Host-side parser for the WPILOG subset [WpiLogWriter] emits. Powers
 * replay and log-content regression tests; AdvantageScope remains the
 * interactive viewer.
 */
class WpiLog private constructor(
    val entries: Map<Int, Entry>,
    private val records: List<Record>,
) {
    data class Entry(val id: Int, val name: String, val type: String)

    class Record(val entryId: Int, val timestampUs: Long, val payload: ByteArray)

    private fun entryId(name: String): Int? = entries.values.firstOrNull { it.name == name }?.id

    fun has(name: String): Boolean = entryId(name) != null

    fun records(name: String): List<Record> {
        val id = entryId(name) ?: return emptyList()
        return records.filter { it.entryId == id }
    }

    fun doubles(name: String): List<Pair<Long, Double>> = records(name).map {
        it.timestampUs to Double.fromBits(readLittleLong(it.payload, 0))
    }

    fun longs(name: String): List<Pair<Long, Long>> = records(name).map {
        it.timestampUs to readLittleLong(it.payload, 0)
    }

    fun booleans(name: String): List<Pair<Long, Boolean>> = records(name).map {
        it.timestampUs to (it.payload[0].toInt() != 0)
    }

    fun strings(name: String): List<Pair<Long, String>> = records(name).map {
        it.timestampUs to String(it.payload, StandardCharsets.UTF_8)
    }

    fun doubleArrays(name: String): List<Pair<Long, DoubleArray>> = records(name).map { record ->
        val count = record.payload.size / 8
        val values = DoubleArray(count) { i -> Double.fromBits(readLittleLong(record.payload, i * 8)) }
        record.timestampUs to values
    }

    companion object {
        fun read(file: File): WpiLog = read(file.readBytes())

        fun read(bytes: ByteArray): WpiLog {
            var pos = 0
            fun u8(): Int = bytes[pos++].toInt() and 0xff
            fun little(n: Int): Long {
                var value = 0L
                for (i in 0 until n) value = value or ((bytes[pos + i].toLong() and 0xff) shl (8 * i))
                pos += n
                return value
            }

            require(
                String(bytes, 0, 6, StandardCharsets.US_ASCII) == "WPILOG",
            ) { "not a WPILOG file" }
            pos = 6
            val version = little(2).toInt()
            require(version == 0x0100) { "unsupported WPILOG version $version" }
            val extraLen = little(4).toInt()
            pos += extraLen

            val entries = HashMap<Int, Entry>()
            val records = ArrayList<Record>()
            while (pos < bytes.size) {
                val header = u8()
                val entryLen = (header and 0x3) + 1
                val sizeLen = ((header shr 2) and 0x3) + 1
                val timeLen = ((header shr 4) and 0x7) + 1
                val entryId = little(entryLen).toInt()
                val payloadSize = little(sizeLen).toInt()
                val timestampUs = little(timeLen)
                val payload = bytes.copyOfRange(pos, pos + payloadSize)
                pos += payloadSize

                if (entryId == 0) {
                    // Control record; we only need entry starts.
                    var c = 0
                    val controlType = payload[c++].toInt()
                    if (controlType == 0) {
                        fun cLittle(n: Int): Long {
                            var value = 0L
                            for (i in 0 until n) value = value or ((payload[c + i].toLong() and 0xff) shl (8 * i))
                            c += n
                            return value
                        }
                        fun cString(): String {
                            val len = cLittle(4).toInt()
                            val s = String(payload, c, len, StandardCharsets.UTF_8)
                            c += len
                            return s
                        }
                        val id = cLittle(4).toInt()
                        val name = cString()
                        val type = cString()
                        entries[id] = Entry(id, name, type)
                    }
                } else {
                    records += Record(entryId, timestampUs, payload)
                }
            }
            return WpiLog(entries, records)
        }
    }
}

private fun readLittleLong(payload: ByteArray, offset: Int): Long {
    var value = 0L
    for (i in 0 until 8) value = value or ((payload[offset + i].toLong() and 0xff) shl (8 * i))
    return value
}
