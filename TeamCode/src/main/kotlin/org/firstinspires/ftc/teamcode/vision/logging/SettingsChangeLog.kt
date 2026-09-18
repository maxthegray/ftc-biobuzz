package org.firstinspires.ftc.teamcode.vision.logging

import java.lang.reflect.Modifier
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Records a config section's values as flight-log events so a WPILOG alone
 * shows which tuning produced the behavior it recorded: every value once, then
 * only the fields that changed.
 *
 * Panels sliders can change a value many times a second, so change events are
 * limited to one per [minIntervalMs]; edits inside the window are merged and
 * written when it expires, and an edit reverted inside the window writes
 * nothing. An edit made in the last [minIntervalMs] before stop may therefore
 * be missing from the log; the tuning file and lab record still hold it.
 */
class SettingsChangeLog(
    private val section: String,
    private val clock: Clock,
    private val sink: (String) -> Unit,
    private val minIntervalMs: Double = 1000.0,
) {
    private var logged: Map<String, String>? = null
    private var pending: Map<String, String>? = null
    private var lastEventNs = Long.MIN_VALUE

    val hasPending: Boolean get() = pending != null

    /** [label] identifies what the values produced, e.g. the detection settings version. */
    fun update(values: Map<String, String>, label: String) {
        val now = clock.nanos()
        val base = logged
        if (base == null) {
            emit("$section ($label): ${values.entries.joinToString(" ") { "${it.key}=${it.value}" }}", values, now)
            return
        }
        pending = values.takeIf { it != base }
        val next = pending ?: return
        if (lastEventNs != Long.MIN_VALUE && (now - lastEventNs) / 1e6 < minIntervalMs) return
        val changes = next.entries.filter { base[it.key] != it.value }
            .joinToString(", ") { "${it.key} ${base[it.key]}→${it.value}" }
        emit("$section changed ($label): $changes", next, now)
    }

    private fun emit(message: String, values: Map<String, String>, now: Long) {
        sink(message)
        logged = values
        pending = null
        lastEventNs = now
    }

    companion object {
        /** Public mutable primitive fields of a config object, in declaration order, shortest formatting. */
        fun valuesOf(config: Any): Map<String, String> {
            val values = LinkedHashMap<String, String>()
            for (field in config.javaClass.declaredFields) {
                if (!Modifier.isPublic(field.modifiers) || Modifier.isFinal(field.modifiers)) continue
                if (!field.type.isPrimitive && field.type != String::class.java) continue
                values[field.name] = field.get(config).toString()
            }
            return values
        }

        fun describe(section: String, config: Any): String =
            "$section: " + valuesOf(config).entries.joinToString(" ") { "${it.key}=${it.value}" }
    }
}
