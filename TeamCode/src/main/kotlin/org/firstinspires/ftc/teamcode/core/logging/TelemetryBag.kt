package org.firstinspires.ftc.teamcode.core.logging

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.math.Velocity
import com.qualcomm.robotcore.util.RobotLog
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.core.Clock

/**
 * Telemetry for the Driver Station and Panels at once. `OpModeBase` flushes it
 * every loop; it only sends every [transmitIntervalMs] (20 Hz), because nobody
 * reads faster and sending is slow. Values are formatted when sent.
 *
 * ```kotlin
 * telemetryBag.section("Drive") {
 *     put("pose", drive.pose)
 *     put("speed", speed, decimals = 1)
 * }
 * ```
 *
 * A section key keeps its latest value; [line]s pile up until sent. A sink
 * that throws is switched off for the rest of the op-mode, so a Panels
 * failure can't take the Driver Station's telemetry with it.
 */
class TelemetryBag internal constructor(
    private val sinks: List<Sink>,
    transmitIntervalMs: Double,
    private val clock: Clock,
) {
    constructor(
        dsTelemetry: Telemetry,
        panels: TelemetryManager,
        transmitIntervalMs: Double = 50.0,
    ) : this(
        listOf(
            object : Sink {
                override fun addLine(text: String) { dsTelemetry.addLine(text) }
                override fun addData(key: String, value: String) { dsTelemetry.addData(key, value) }
                override fun update() { dsTelemetry.update() }
            },
            object : Sink {
                override fun addLine(text: String) { panels.addLine(text) }
                override fun addData(key: String, value: String) { panels.addData(key, value) }
                override fun update() { panels.update() }
            },
        ),
        transmitIntervalMs,
        Clock.SYSTEM,
    )

    interface Sink {
        fun addLine(text: String)
        fun addData(key: String, value: String)
        fun update()
    }

    private val transmitIntervalNs = (transmitIntervalMs * 1_000_000.0).toLong()
    private var lastTransmitNs = Long.MIN_VALUE
    private val enabledSinks = BooleanArray(sinks.size) { true }
    private val sections = linkedMapOf<String, LinkedHashMap<String, Any?>>()
    private val lines = mutableListOf<String>()

    fun section(name: String, block: Section.() -> Unit) {
        Section(sections.getOrPut(name) { LinkedHashMap() }).block()
    }

    /** A free-form line, sent below the sections. */
    fun line(text: String) {
        lines += text
    }

    /** Sends everything if [transmitIntervalMs] has passed; returns true when it did. */
    fun flush(): Boolean {
        val now = clock.nanos()
        if (lastTransmitNs != Long.MIN_VALUE && now - lastTransmitNs < transmitIntervalNs) return false
        lastTransmitNs = now
        try {
            for ((name, entries) in sections) {
                if (entries.isEmpty()) continue
                forEachSink { it.addLine("== $name ==") }
                for ((key, value) in entries) {
                    val text = formatValue(value)
                    forEachSink { it.addData(key, text) }
                }
            }
            for (text in lines) forEachSink { it.addLine(text) }
            forEachSink { it.update() }
        } finally {
            for (entries in sections.values) entries.clear()
            lines.clear()
        }
        return true
    }

    private inline fun forEachSink(action: (Sink) -> Unit) {
        sinks.forEachIndexed { i, sink ->
            if (!enabledSinks[i]) return@forEachIndexed
            try {
                action(sink)
            } catch (t: Throwable) {
                enabledSinks[i] = false
                RobotLog.ee("TelemetryBag", t, "Telemetry sink disabled")
            }
        }
    }

    class Section internal constructor(private val entries: LinkedHashMap<String, Any?>) {
        fun put(key: String, value: Any?) {
            entries[key] = value
        }

        fun put(key: String, value: Double, decimals: Int = 3) {
            entries[key] = Decimal(value, decimals)
        }
    }

    private class Decimal(val value: Double, val decimals: Int)

    companion object {
        // Locale pinned so a hub or laptop locale can't switch to comma decimals.
        internal fun formatValue(value: Any?): String = when (value) {
            null -> "null"
            is Decimal -> "%.${value.decimals}f".format(Locale.US, value.value)
            is Pose -> "(%.2f, %.2f, %.1f°)".format(Locale.US, value.x(), value.y(), Math.toDegrees(value.heading()))
            is Velocity -> "(%.2f, %.2f, %.1f°/s)".format(Locale.US, value.vx, value.vy, Math.toDegrees(value.omega))
            is Vector2D -> "(%.2f, %.2f)".format(Locale.US, value.x(), value.y())
            is Double, is Float -> "%.3f".format(Locale.US, value)
            else -> value.toString()
        }
    }
}
