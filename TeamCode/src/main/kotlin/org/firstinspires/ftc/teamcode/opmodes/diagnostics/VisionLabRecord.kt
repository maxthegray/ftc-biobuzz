package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.Executor
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.SettingsChangeLog
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.vision.BallCameraConfig

/**
 * A reviewable text record of a vision tuning session: the live config values,
 * which of them differ from the compiled values (with the edit that would
 * adopt each one), and the measurements that justify them.
 *
 * Records land in `/sdcard/FIRST/lab-records/` and are meant to be pulled
 * (`adb pull /sdcard/FIRST/lab-records/. lab-records/`), annotated, and committed under `lab-records/`.
 */
object VisionLabRecord {

    data class ConfigSection(
        /** The config object's name, e.g. `BallCameraConfig`. */
        val name: String,
        val current: Map<String, String>,
        val compiledDefaults: Map<String, String>,
    )

    fun section(config: Any, compiledDefaults: Map<String, String>) =
        ConfigSection(config.javaClass.simpleName, SettingsChangeLog.valuesOf(config), compiledDefaults)

    fun fileName(opModeName: String, wallClockMs: Long): String {
        val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date(wallClockMs))
        val slug = opModeName.lowercase(Locale.US).replace(Regex("[^a-z0-9]+"), "-").trim('-')
        return "$slug-$stamp.txt"
    }

    fun build(
        opModeName: String,
        wallClockMs: Long,
        sections: List<ConfigSection>,
        measurements: List<Pair<String, String>>,
    ): String = buildString {
        val stamp = SimpleDateFormat("yyyy-MM-dd HH:mm:ss Z", Locale.US).format(Date(wallClockMs))
        appendLine("# BIOBUZZ vision lab record")
        appendLine("# OpMode: $opModeName")
        appendLine("# Control Hub wall clock: $stamp (check it; the hub clock is often wrong)")
        appendLine("# Fill in before committing:")
        appendLine("#   who / where / lighting:")
        appendLine("#   what was verified (balls, distances, motion):")
        appendLine("#   accept into the code? (yes/no, why):")
        appendLine()

        val adoptions = ArrayList<String>()
        for (section in sections) {
            appendLine("[config ${section.name}]")
            for ((field, value) in section.current) {
                val default = section.compiledDefaults[field]
                append("${section.name}.$field=$value")
                if (default != null && default != value) {
                    append("    # TUNED; compiled value $default")
                    adoptions += "${section.name}.$field = $value"
                }
                appendLine()
            }
            appendLine()
        }

        appendLine("[adopt into the code]")
        if (adoptions.isEmpty()) {
            appendLine("# nothing differs from the compiled values")
        } else {
            appendLine("# set these field initializers in the config objects")
            for (line in adoptions) appendLine(line)
        }
        appendLine()

        appendLine("[measured]")
        for ((key, value) in measurements) appendLine("$key=$value")
    }
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

/**
 * Saves [VisionLabRecord]s from a diagnostic OpMode and reports completion in
 * telemetry and the flight log. A subsystem only so its writer thread is shut
 * down with the OpMode; it touches no hardware.
 */
internal class VisionLabRecorder(
    private val opModeName: String,
    private val writer: LabRecordWriter = LabRecordWriter(),
    /** Further config objects to record, with their compiled values. */
    private val extraSections: List<Pair<Any, Map<String, String>>> = emptyList(),
) : SubsystemBase("VisionLabRecord") {

    private var reportedWritten = 0
    private var reportedError: String? = null

    fun save(measurements: List<Pair<String, String>>) {
        val now = System.currentTimeMillis()
        val contents = VisionLabRecord.build(
            opModeName = opModeName,
            wallClockMs = now,
            sections = listOf(
                VisionLabRecord.section(BallCameraConfig, BallCameraConfig.compiledDefaults),
                VisionLabRecord.section(VisionDiagnosticsConfig, VisionDiagnosticsConfig.compiledDefaults),
            ) + extraSections.map { (config, defaults) -> VisionLabRecord.section(config, defaults) },
            measurements = measurements,
        )
        writer.submit(VisionLabRecord.fileName(opModeName, now), contents)
    }

    /** Call from the robot thread each loop; records completed writes as flight-log events. */
    fun poll(robot: Robot) {
        val status = writer.status
        if (status.written != reportedWritten) {
            reportedWritten = status.written
            robot.recordEvent("VISION LAB RECORD saved: ${status.lastFile}")
        }
        if (status.lastError != null && status.lastError != reportedError) {
            reportedError = status.lastError
            robot.recordEvent("VISION LAB RECORD failed: ${status.lastError}")
        }
    }

    override fun health(): String {
        val status = writer.status
        return when {
            status.pending > 0 -> "saving…"
            status.lastError != null -> "last save failed: ${status.lastError}"
            status.lastFile != null -> "saved ${status.written}: ${status.lastFile}"
            else -> "A saves a record (after START)"
        }
    }

    override fun logState(log: StateLog) {
        log.put("written", writer.status.written.toLong())
    }

    override fun stop() {
        writer.close()
    }
}
