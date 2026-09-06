package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.util.RobotLog
import java.io.File
import java.io.IOException
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
 * ConfigStore.register("lift", LiftConfig, LiftConfig::resetDefaults)
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
 * Files whose schema does not match [RobotConfig.CONFIG_SCHEMA] are ignored,
 * preventing a season fork from inheriting stale tuning by accident.
 */
object ConfigStore {

    internal const val SCHEMA_KEY = "config.schema"

    /** Overridable for host tests; null disables persistence entirely. */
    internal var file: File? = File("/sdcard/FIRST/config/tuning.properties")
    internal var schemaId: String = RobotConfig.CONFIG_SCHEMA

    private class Section(val config: Any, val resetDefaults: () -> Unit)

    private val sections = LinkedHashMap<String, Section>()
    private var lastPersisted: Map<String, String>? = null

    /**
     * Register a config object under [section]. Re-registering replaces both arguments.
     * [resetDefaults] must restore its tunable fields from compiled constants, not
     * a snapshot of live values; Panels may already have changed those values.
     */
    fun register(section: String, config: Any, resetDefaults: () -> Unit) {
        require(section.isNotBlank() && '.' !in section && '=' !in section) {
            "section must be a simple name, got \"$section\""
        }
        sections[section] = Section(config, resetDefaults)
    }

    /**
     * Reset every registered object, then apply matching-schema overrides.
     * Missing files/keys and invalid values leave compiled defaults in place,
     * including in a warm process. Unknown keys are retained when persisting.
     */
    fun loadFromDisk() {
        val target = file ?: return
        try {
            for (section in sections.values) section.resetDefaults()
            for ((key, value) in readMatchingValues(target)) {
                applyValue(key, value)
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
    fun persistIfDirty(): Boolean = persistIfDirty(File::renameTo)

    internal fun persistIfDirty(replaceFile: (File, File) -> Boolean): Boolean {
        val target = file ?: return false
        val current = snapshot()
        if (current == lastPersisted) return false
        val tmp = File(target.path + ".tmp")
        try {
            // Read on each dirty write so partial registrations (even without a
            // prior load) and hand-edited unknown keys survive this opmode.
            val merged = readMatchingValues(target) + current
            target.parentFile?.mkdirs()
            tmp.writeText(
                buildString {
                    appendLine("# ftc-biobuzz live-tuning values. Written by ConfigStore;")
                    appendLine("# loaded into config objects at every op-mode init.")
                    appendLine("# Delete this file to fall back to compiled defaults.")
                    appendLine("$SCHEMA_KEY=$schemaId")
                    for ((key, value) in merged) appendLine("$key=$value")
                },
            )
            if (!replaceFile(tmp, target)) {
                throw IOException("Could not replace tuning file $target")
            }
            lastPersisted = current
            return true
        } catch (t: Throwable) {
            log(t, "Failed to persist tuning config")
            return false
        } finally {
            if (tmp.isFile) tmp.delete()
        }
    }

    /** Current values of every registered field, keyed `<section>.<field>`. */
    internal fun snapshot(): Map<String, String> {
        val values = LinkedHashMap<String, String>()
        for ((section, registration) in sections) {
            val config = registration.config
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

    private fun readMatchingValues(target: File): Map<String, String> {
        if (!target.exists()) return emptyMap()
        val persisted = LinkedHashMap<String, String>()
        for (line in target.readLines()) {
            val trimmed = line.trim()
            if (trimmed.isEmpty() || trimmed.startsWith("#")) continue
            val eq = trimmed.indexOf('=')
            if (eq <= 0) continue
            persisted[trimmed.substring(0, eq).trim()] = trimmed.substring(eq + 1).trim()
        }
        if (persisted.remove(SCHEMA_KEY) != schemaId) return emptyMap()
        return persisted
    }

    private fun applyValue(key: String, raw: String) {
        val dot = key.indexOf('.')
        if (dot <= 0) return
        val config = sections[key.substring(0, dot)]?.config ?: return
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
