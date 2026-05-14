package org.firstinspires.ftc.teamcode.core.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.geometry.Pose
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
 * The base [OpModeBase] calls [flush] at the end of every loop — don't call
 * it manually unless you know what you are doing.
 */
class TelemetryBag(
    private val dsTelemetry: Telemetry,
    private val panels: TelemetryManager,
) {
    private val sections = linkedMapOf<String, LinkedHashMap<String, String>>()

    /**
     * Open a section. Repeated keys within a section overwrite — so this is
     * safe to call multiple times per loop with the same section name.
     */
    fun section(name: String, block: Section.() -> Unit) {
        val entries = sections.getOrPut(name) { linkedMapOf() }
        Section(entries).block()
    }

    /** Append a free-form line. These are flushed below the structured sections. */
    fun line(text: String) {
        loose += text
    }

    private val loose = mutableListOf<String>()

    /**
     * Write the accumulated content to both the Driver Station and Panels,
     * then clear the buffer. Called by [org.firstinspires.ftc.teamcode.core.runtime.OpModeBase]
     * at the end of every loop tick.
     */
    fun flush() {
        for ((section, entries) in sections) {
            dsTelemetry.addLine("== $section ==")
            panels.addLine("== $section ==")
            for ((k, v) in entries) {
                dsTelemetry.addData(k, v)
                panels.addData(k, v)
            }
        }
        for (line in loose) {
            dsTelemetry.addLine(line)
            panels.addLine(line)
        }
        dsTelemetry.update()
        panels.update()
        sections.clear()
        loose.clear()
    }

    class Section internal constructor(private val entries: LinkedHashMap<String, String>) {
        fun put(key: String, value: Any?) {
            entries[key] = formatValue(value)
        }

        fun put(key: String, value: Double, decimals: Int = 3) {
            entries[key] = "%.${decimals}f".format(value)
        }
    }

    companion object {
        internal fun formatValue(value: Any?): String = when (value) {
            null -> "null"
            is Pose -> "(%.2f, %.2f, %.1f°)".format(
                value.x,
                value.y,
                Math.toDegrees(value.heading),
            )
            is Double -> "%.3f".format(value)
            is Float -> "%.3f".format(value)
            else -> value.toString()
        }
    }
}
