package org.firstinspires.ftc.teamcode.core.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.geometry.Pose
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Owns the per-tick telemetry buffer. One [TelemetryBag] is shared between
 * the FTC Driver Station telemetry and the Panels dashboard telemetry so
 * op-modes only log things once.
 *
 * Usage from inside an op-mode's `onLoop`:
 *
 * ```kotlin
 * telemetryBag.section("Drive") {
 *     put("pose", drive.pose)
 *     put("loopHz", robot.loopHz)
 *     put("isBusy", drive.isFollowing)
 * }
 * ```
 *
 * The base [org.firstinspires.ftc.teamcode.core.runtime.OpModeBase] calls
 * [flush] at the end of every loop — don't call it manually unless you know
 * what you are doing.
 *
 * ## Loop-time behaviour
 *
 * [flush] is called every tick but only *transmits* every [transmitIntervalMs]
 * (default 50 ms ≈ 20 Hz). Pushing the Panels websocket and rebuilding the
 * Driver Station telemetry at the full loop rate is pure overhead — nobody can
 * read a dashboard at 100+ Hz — and the periodic transmission shows up as a
 * loop-time spike. Between transmissions, [put] only stores a value reference,
 * so string formatting also runs at the throttled rate, not per tick.
 *
 * Section maps, [Section] wrappers, and the per-key [FormattedDouble] holders
 * are all reused across ticks, so a steady-state loop allocates effectively
 * nothing here — fewer young-gen allocations means fewer GC pauses, which is
 * what actually destabilises loop times on the Control Hub.
 *
 * Semantics: section entries are "current state" — repeated [put]s with the
 * same key overwrite, and the latest value at transmit time is the one sent.
 * [line] entries are "events" — they accumulate across the throttle window so
 * nothing logged between transmissions is dropped.
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
        listOf(TelemetrySink(dsTelemetry), PanelsSink(panels)),
        transmitIntervalMs,
        Clock.SYSTEM,
    )

    /** One transmission target. Production sinks adapt the DS [Telemetry] and Panels. */
    interface Sink {
        fun addLine(text: String)
        fun addData(key: String, value: String)
        fun update()
    }

    private class TelemetrySink(private val telemetry: Telemetry) : Sink {
        override fun addLine(text: String) { telemetry.addLine(text) }
        override fun addData(key: String, value: String) { telemetry.addData(key, value) }
        override fun update() { telemetry.update() }
    }

    private class PanelsSink(private val panels: TelemetryManager) : Sink {
        override fun addLine(text: String) { panels.addLine(text) }
        override fun addData(key: String, value: String) { panels.addData(key, value) }
        override fun update() { panels.update() }
    }

    private val transmitIntervalNs = (transmitIntervalMs * 1_000_000.0).toLong()
    private var lastTransmitNs = Long.MIN_VALUE

    private val sections = linkedMapOf<String, SectionData>()
    private val loose = mutableListOf<String>()

    /**
     * Open a section. Repeated keys within a section overwrite — so this is
     * safe to call multiple times per loop with the same section name.
     */
    fun section(name: String, block: Section.() -> Unit) {
        val data = sections.getOrPut(name) { SectionData() }
        data.section.block()
    }

    /** Append a free-form line. These are flushed below the structured sections. */
    fun line(text: String) {
        loose += text
    }

    /**
     * Called once per loop by [org.firstinspires.ftc.teamcode.core.runtime.OpModeBase].
     * Transmits to the Driver Station and Panels only once every
     * [transmitIntervalNs]; on throttled ticks it returns immediately, leaving
     * the accumulated state to be overwritten by the next [section] / [line]
     * calls and sent on the next real transmission.
     *
     * @return true when telemetry was actually transmitted.
     */
    fun flush(): Boolean {
        val now = clock.nanos()
        if (lastTransmitNs != Long.MIN_VALUE && now - lastTransmitNs < transmitIntervalNs) return false
        lastTransmitNs = now

        for ((name, data) in sections) {
            if (data.entries.isEmpty()) continue
            for (sink in sinks) sink.addLine("== $name ==")
            for ((k, v) in data.entries) {
                val formatted = formatValue(v)
                for (sink in sinks) sink.addData(k, formatted)
            }
        }
        for (text in loose) {
            for (sink in sinks) sink.addLine(text)
        }
        for (sink in sinks) sink.update()

        // Reuse the section/entry maps — clear contents, not the structure.
        for (data in sections.values) data.entries.clear()
        loose.clear()
        return true
    }

    /** A section's reused entry map plus the reused [Section] wrapper over it. */
    private class SectionData {
        val entries = LinkedHashMap<String, Any?>()
        val section = Section(entries)
    }

    class Section internal constructor(private val entries: LinkedHashMap<String, Any?>) {
        /** Store a value as-is; it is formatted at transmit time, not now. */
        fun put(key: String, value: Any?) {
            if (value is Double) {
                // Route through the holder-reusing path so a boxed Double never
                // replaces a reusable FormattedDouble for the same key.
                put(key, value, decimals = 3)
            } else {
                entries[key] = value
            }
        }

        /**
         * Store a double with a decimal-place hint. The [FormattedDouble] holder
         * is reused across ticks — after the first loop this allocates nothing.
         */
        fun put(key: String, value: Double, decimals: Int = 3) {
            val existing = entries[key]
            if (existing is FormattedDouble) {
                existing.value = value
                existing.decimals = decimals
            } else {
                entries[key] = FormattedDouble(value, decimals)
            }
        }
    }

    /** Mutable, reused holder for a deferred-format double. */
    private class FormattedDouble(var value: Double, var decimals: Int)

    companion object {
        // Locale pinned so output never switches to comma decimals on a
        // non-US-locale JVM (host-side tests, hub locale changes).
        internal fun formatValue(value: Any?): String = when (value) {
            null -> "null"
            is FormattedDouble -> "%.${value.decimals}f".format(Locale.US, value.value)
            is Pose -> "(%.2f, %.2f, %.1f°)".format(
                Locale.US,
                value.x,
                value.y,
                Math.toDegrees(value.heading),
            )
            is Double -> "%.3f".format(Locale.US, value)
            is Float -> "%.3f".format(Locale.US, value)
            else -> value.toString()
        }
    }
}
