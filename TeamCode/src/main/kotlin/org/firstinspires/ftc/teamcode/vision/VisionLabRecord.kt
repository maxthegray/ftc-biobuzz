package org.firstinspires.ftc.teamcode.vision

import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executor
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore

/**
 * A reviewable text record of a vision tuning session: the live config values
 * exactly as ConfigStore would persist them, which of them differ from the
 * compiled defaults (with the constant edit that would adopt each one), and
 * the measurements that justify them.
 *
 * Records land in `/sdcard/FIRST/lab-records/` and are meant to be pulled
 * (`make pull-lab-records`), annotated, and committed under `lab-records/`.
 */
object VisionLabRecord {

    data class ConfigSection(
        val section: String,
        /** Field name → value, as registered in ConfigStore. */
        val current: Map<String, String>,
        val compiledDefaults: Map<String, Any>,
    )

    fun sectionFromStore(section: String, compiledDefaults: Map<String, Any>): ConfigSection {
        val prefix = "$section."
        val current = ConfigStore.snapshot()
            .filterKeys { it.startsWith(prefix) }
            .mapKeys { it.key.removePrefix(prefix) }
        return ConfigSection(section, current, compiledDefaults)
    }

    fun fileName(opModeName: String, wallClockMs: Long): String {
        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date(wallClockMs))
        val slug = opModeName.lowercase(Locale.US).replace(Regex("[^a-z0-9]+"), "-").trim('-')
        return "$slug-$stamp.txt"
    }

    fun build(
        opModeName: String,
        wallClockMs: Long,
        configSchema: String,
        sections: List<ConfigSection>,
        measurements: List<Pair<String, String>>,
    ): String = buildString {
        val stamp = SimpleDateFormat("yyyy-MM-dd HH:mm:ss Z", Locale.US).format(Date(wallClockMs))
        appendLine("# BIOBUZZ vision lab record")
        appendLine("# OpMode: $opModeName")
        appendLine("# Control Hub wall clock: $stamp (check it; the hub clock is often wrong)")
        appendLine("# Config schema: $configSchema")
        appendLine("# Fill in before committing:")
        appendLine("#   who / where / lighting:")
        appendLine("#   what was verified (balls, distances, motion):")
        appendLine("#   accept as compiled defaults? (yes/no, why):")
        appendLine()

        val adoptions = ArrayList<String>()
        for (section in sections) {
            appendLine("[config ${section.section}]  # key=value exactly as in tuning.properties")
            for ((field, value) in section.current) {
                val default = section.compiledDefaults[field]
                val tuned = default != null && ConfigStore.formatValue(default) != value
                append("${section.section}.$field=$value")
                if (tuned) {
                    append("    # TUNED; compiled default ${readable(default, ConfigStore.formatValue(default))}")
                    adoptions += "${section.section}: DEFAULT_${upperSnake(field)} = ${readable(default, value)}"
                } else if (default == null) {
                    append("    # no compiled default listed")
                }
                appendLine()
            }
            appendLine()
        }

        appendLine("[adopt as compiled defaults]")
        if (adoptions.isEmpty()) {
            appendLine("# nothing differs from the compiled defaults")
        } else {
            appendLine("# edit these private constants, then delete the matching keys from tuning.properties")
            for (line in adoptions) appendLine(line)
        }
        appendLine()

        appendLine("[measured]")
        for ((key, value) in measurements) appendLine("$key=$value")
    }

    /** Values are persisted with full precision; show them in their shortest form. */
    private fun readable(typeHint: Any?, raw: String): String = when (typeHint) {
        is Double -> raw.toDoubleOrNull()?.toString() ?: raw
        is Float -> raw.toFloatOrNull()?.toString() ?: raw
        else -> raw
    }

    internal fun upperSnake(camel: String): String =
        camel.replace(Regex("([a-z0-9])([A-Z])"), "$1_$2").uppercase(Locale.US)
}

data class LabRecordStatus(
    val pending: Int = 0,
    val written: Int = 0,
    val lastFile: String? = null,
    val lastError: String? = null,
)

/**
 * Writes lab records off the robot thread, atomically (temp file + rename).
 * The robot loop polls [status] to report completion.
 */
class LabRecordWriter(
    private val directory: File = File("/sdcard/FIRST/lab-records"),
    private val executor: Executor = defaultExecutor(),
) {
    private val statusRef = AtomicReference(LabRecordStatus())

    val status: LabRecordStatus get() = statusRef.get()

    fun submit(fileName: String, contents: String) {
        statusRef.updateAndGet { it.copy(pending = it.pending + 1) }
        executor.execute {
            val target = File(directory, fileName)
            val tmp = File(directory, "$fileName.tmp")
            try {
                directory.mkdirs()
                tmp.writeText(contents)
                if (!tmp.renameTo(target)) error("could not rename $tmp to $target")
                statusRef.updateAndGet { it.copy(pending = it.pending - 1, written = it.written + 1, lastFile = target.path) }
            } catch (t: Throwable) {
                if (tmp.isFile) tmp.delete()
                statusRef.updateAndGet {
                    it.copy(pending = it.pending - 1, lastError = "${t.javaClass.simpleName}: ${t.message}")
                }
            }
        }
    }

    /** Lets queued writes finish in the background; accepts no new work. */
    fun close() {
        (executor as? ExecutorService)?.shutdown()
    }

    private companion object {
        fun defaultExecutor(): ExecutorService = Executors.newSingleThreadExecutor { runnable ->
            Thread(runnable, "vision-lab-record").apply { isDaemon = true }
        }
    }
}
