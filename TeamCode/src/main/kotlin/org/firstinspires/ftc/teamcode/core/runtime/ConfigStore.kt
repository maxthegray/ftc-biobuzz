package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.util.RobotLog
import java.io.File
import java.lang.reflect.Field
import java.lang.reflect.Modifier
import java.util.Locale

/**
 * File-backed persistence for live-tunable config objects.
 *
 * Panels tunes `@Configurable` objects by writing their `@JvmField` statics
 * — which die with the process. This store closes that hole: registered
 * config objects are snapshotted to
 * `/sdcard/FIRST/config/tuning.properties`, and the file is reloaded into
 * the objects at every op-mode init. Tune at the practice field, power
 * cycle, values survive. It also removes the need to `@Pinned` config
 * objects against Sloth hot reloads — a reload resets the statics, but the
 * next op-mode init restores them from disk — so config *code* is
 * hot-reloadable again.
 *
 * Usage: the framework registers `DriveConfig` / `LocalizerConfig` itself
 * (see [OpModeBase]); season forks add their own in `configure()`:
 *
 * ```kotlin
 * ConfigStore.register("lift", LiftConfig)
 * ```
 *
 * Only public `@JvmField` mutable fields of primitive-ish types (Double,
 * Float, Int, Long, Boolean, String) are persisted, keyed
 * `<section>.<field>`. Values rejected by a config object's own `safe*`
 * guards are still guarded — the store does not validate semantics, it
 * just round-trips what Panels wrote.
 *
 * To hand-edit: `adb pull`/`push` the file, or delete it to fall back to
 * compiled defaults. Compiled defaults also apply for any key missing from
 * the file, so adding a new field never requires touching the file.
 */
object ConfigStore {

    /** Overridable for host tests; null disables persistence entirely. */
    internal var file: File? = File("/sdcard/FIRST/config/tuning.properties")

    private val sections = LinkedHashMap<String, Any>()
    private var lastPersisted: Map<String, String>? = null

    /** Register a config object under [section]. Idempotent; re-registering replaces. */
    fun register(section: String, config: Any) {
        require(section.isNotBlank() && '.' !in section && '=' !in section) {
            "section must be a simple name, got \"$section\""
        }
        sections[section] = config
    }

    /**
     * Apply persisted overrides to every registered object. Missing file,
     * unknown keys, and unparseable values are ignored — compiled defaults
     * win wherever the file has nothing better to say.
     */
    fun loadFromDisk() {
        val target = file ?: return
        try {
            if (target.exists()) {
                for (line in target.readLines()) {
                    val trimmed = line.trim()
                    if (trimmed.isEmpty() || trimmed.startsWith("#")) continue
                    val eq = trimmed.indexOf('=')
                    if (eq <= 0) continue
                    applyValue(trimmed.substring(0, eq).trim(), trimmed.substring(eq + 1).trim())
                }
            }
        } catch (t: Throwable) {
            log(t, "Failed to load tuning config")
        }
        // What's in memory now is the baseline — don't rewrite an unchanged file.
        lastPersisted = snapshot()
    }

    /**
     * Persist the current values if anything changed since the last load or
     * persist. Atomic (tmp + rename), best-effort, cheap when clean (one
     * reflective snapshot). Returns true when a write happened.
     */
    fun persistIfDirty(): Boolean {
        val target = file ?: return false
        val current = snapshot()
        if (current == lastPersisted) return false
        try {
            target.parentFile?.mkdirs()
            val tmp = File(target.path + ".tmp")
            tmp.writeText(
                buildString {
                    appendLine("# ftc-starter live-tuning values. Written by ConfigStore;")
                    appendLine("# loaded into config objects at every op-mode init.")
                    appendLine("# Delete this file to fall back to compiled defaults.")
                    for ((key, value) in current) appendLine("$key=$value")
                },
            )
            if (!tmp.renameTo(target)) {
                target.delete()
                tmp.renameTo(target)
            }
            lastPersisted = current
            return true
        } catch (t: Throwable) {
            log(t, "Failed to persist tuning config")
            return false
        }
    }

    /** Current values of every registered field, keyed `<section>.<field>`. */
    internal fun snapshot(): Map<String, String> {
        val values = LinkedHashMap<String, String>()
        for ((section, config) in sections) {
            for (field in tunableFields(config)) {
                values["$section.${field.name}"] = formatValue(field.get(config))
            }
        }
        return values
    }

    /** Host-test hook: forget registrations and baseline (the file is untouched). */
    internal fun reset() {
        sections.clear()
        lastPersisted = null
    }

    private fun applyValue(key: String, raw: String) {
        val dot = key.indexOf('.')
        if (dot <= 0) return
        val config = sections[key.substring(0, dot)] ?: return
        val fieldName = key.substring(dot + 1)
        val field = tunableFields(config).firstOrNull { it.name == fieldName } ?: return
        try {
            when (field.type) {
                java.lang.Double.TYPE -> raw.toDoubleOrNull()
                    ?.takeIf { it.isFinite() }
                    ?.let { field.setDouble(config, it) }
                java.lang.Float.TYPE -> raw.toFloatOrNull()
                    ?.takeIf { it.isFinite() }
                    ?.let { field.setFloat(config, it) }
                java.lang.Integer.TYPE -> raw.toIntOrNull()?.let { field.setInt(config, it) }
                java.lang.Long.TYPE -> raw.toLongOrNull()?.let { field.setLong(config, it) }
                java.lang.Boolean.TYPE -> raw.toBooleanStrictOrNull()?.let { field.setBoolean(config, it) }
                String::class.java -> field.set(config, raw)
            }
        } catch (t: Throwable) {
            log(t, "Failed to apply tuning value $key")
        }
    }

    // Kotlin `object` @JvmField vars compile to *static* fields (which is
    // what Panels tunes), so statics are included; `INSTANCE` and private
    // DEFAULT_* constants fall out via the public + non-final filters.
    private fun tunableFields(config: Any): List<Field> =
        config.javaClass.declaredFields.filter { field ->
            Modifier.isPublic(field.modifiers) &&
                !Modifier.isFinal(field.modifiers) &&
                (field.type.isPrimitive || field.type == String::class.java)
        }

    private fun formatValue(value: Any?): String = when (value) {
        is Double -> "%.17g".format(Locale.US, value)
        is Float -> "%.9g".format(Locale.US, value)
        else -> value.toString()
    }

    private fun log(t: Throwable, message: String) {
        try {
            RobotLog.ee("ConfigStore", t, message)
        } catch (_: Throwable) {
            // Host-side tests stub Android logging.
        }
    }
}
