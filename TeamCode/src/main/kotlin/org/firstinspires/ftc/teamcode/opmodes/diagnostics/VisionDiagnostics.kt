package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.LoopPhase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag
import org.firstinspires.ftc.teamcode.vision.BallCameraSubsystem
import org.firstinspires.ftc.teamcode.vision.BallPreviewMode
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig
import org.firstinspires.ftc.teamcode.vision.BallVisionSettings
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.HiveCellReadiness
import org.firstinspires.ftc.teamcode.vision.LabRecordWriter
import org.firstinspires.ftc.teamcode.vision.TagSightingTracker
import org.firstinspires.ftc.teamcode.vision.VisionDiagnosticsConfig
import org.firstinspires.ftc.teamcode.vision.VisionLabRecord

/** Wiring and telemetry shared by the Limelight AprilTag Test and Ball Tracking Test. */
internal object VisionDiagnostics {

    val limelightRequirement = Preflight.Requirement(LimelightSubsystem.DEFAULT_HARDWARE_NAME, Limelight3A::class.java)
    val ballCameraRequirement = Preflight.Requirement(BallCameraSubsystem.DEFAULT_HARDWARE_NAME, WebcamName::class.java)

    /** Restart-only choices, captured once in `configure()`. */
    data class Startup(
        val runBothCameras: Boolean,
        val tagPipelineIndex: Int,
        val pollRateHz: Int,
        val maxResultAgeMs: Long,
    ) {
        companion object {
            fun fromConfig() = Startup(
                runBothCameras = VisionDiagnosticsConfig.runBothCameras,
                tagPipelineIndex = VisionDiagnosticsConfig.safeTagPipelineIndex,
                pollRateHz = VisionDiagnosticsConfig.safePollRateHz,
                maxResultAgeMs = VisionDiagnosticsConfig.safeMaxResultAgeMs,
            )
        }
    }

    /**
     * Registers both vision config sections and loads them now, because
     * `configure()` decides which cameras to open from them; OpModeBase's own
     * load runs only after `configure()` returns.
     */
    fun registerConfigsAndLoad(): Startup {
        ConfigStore.register("visionDiagnostics", VisionDiagnosticsConfig, VisionDiagnosticsConfig::resetDefaults)
        ConfigStore.register("ballVision", BallVisionConfig, BallVisionConfig::resetDefaults)
        ConfigStore.loadFromDisk()
        return Startup.fromConfig()
    }

    fun limelight(startup: Startup) = LimelightSubsystem(
        pipelineIndex = startup.tagPipelineIndex,
        pollRateHz = startup.pollRateHz,
        maxResultAgeMs = startup.maxResultAgeMs,
    )

    fun restartSection(bag: TelemetryBag, startup: Startup, camera: BallCameraSubsystem?) {
        val pending = ArrayList<String>()
        val requested = Startup.fromConfig()
        if (requested != startup) pending += "visionDiagnostics (both cameras / Limelight pipeline, rate, age)"
        if (camera != null && camera.restartRequired) pending += "ballVision stream ${camera.requestedStream}"
        if (pending.isEmpty()) return
        bag.section("RESTART REQUIRED") {
            put("changed", pending.joinToString("; "))
            put("running", "both cameras=${startup.runBothCameras}, LL pipeline ${startup.tagPipelineIndex}" +
                (camera?.runningStream?.let { ", camera $it" } ?: ""))
        }
    }

    // ---------------------------------------------------------------- Limelight

    fun limelightSections(
        bag: TelemetryBag,
        limelight: LimelightSubsystem,
        sightings: TagSightingTracker,
        nowNs: Long,
        detailed: Boolean,
    ) {
        bag.section("Limelight") {
            put("state", limelight.health())
            put("pipeline", "${limelight.activePipelineIndex} (${limelight.pipelineType}); expected ${limelight.pipelineIndex}")
            put("switch accepted", limelight.pipelineSwitchAccepted)
            put("result fresh", limelight.resultFresh)
            put("receipt age ms (CH clock)", limelight.resultAgeMs)
            put("frame age ms (first receipt)", limelight.frameAgeMs, decimals = 1)
            put("est. capture age ms", limelight.estimatedCaptureAgeMs, decimals = 1)
            put("capture+targeting ms (LL)", limelight.captureLatencyMs + limelight.targetingLatencyMs, decimals = 1)
            put("new frames Hz", limelight.resultRateHz, decimals = 1)
            put("frame ts (LL clock ms)", limelight.limelightTimestampMs, decimals = 0)
            put("stale ticks", limelight.staleTickCount)
        }
        if (!detailed) {
            bag.section("Tags (summary)") {
                put("visible IDs", limelight.fiducials.joinToString(",") { it.id.toString() }.ifEmpty { "none" })
            }
            return
        }
        bag.section("Tags Now") {
            put("frame", "${limelight.fiducials.size} tag(s); fresh=${limelight.resultFresh}")
            for (fiducial in limelight.fiducials) put("ID ${fiducial.id}", describeFiducial(fiducial))
        }
        bag.section("Tags Seen This Run") {
            val all = sightings.all()
            if (all.isEmpty()) put("none", "no tag in any fresh frame yet")
            for (s in all) {
                val meaning = s.catalogTag?.meaning ?: "NOT A BIOBUZZ TAG"
                put("ID ${s.id}", "$meaning; last ${fmt(s.ageMs(nowNs), 0)} ms ago; ${s.framesSeen}/${sightings.framesObserved} frames")
            }
        }
        bag.section("HIVE Cells") {
            for (cell in BiobuzzAprilTags.cells) {
                val ids = BiobuzzAprilTags.tagsOf(cell).joinToString(",") { it.id.toString() }
                val last = sightings.latestFor(cell)?.let { "last tag ${fmt(it.ageMs(nowNs), 0)} ms ago" } ?: "no tag seen"
                put("${cell.stickerLabel} ($ids)", "$last; readiness ${HiveCellReadiness.NOT_INFERRED}")
            }
            put("note", "tags identify a CELL; they do not show which CELL is up or ready")
        }
    }

    private fun describeFiducial(f: LimelightFiducial): String {
        val tag = BiobuzzAprilTags.lookup(f.id)
        val family = if (BiobuzzAprilTags.isSeasonFamily(f.family)) "" else " FAMILY ${f.family}?"
        val pose = f.targetPoseCameraSpace?.let {
            "cam xyz (${fmt(it.xMeters, 2)},${fmt(it.yMeters, 2)},${fmt(it.zMeters, 2)}) m, " +
                "|d| ${fmt(it.distanceMeters, 2)} m, rpy (${fmt(it.rollDegrees, 1)},${fmt(it.pitchDegrees, 1)},${fmt(it.yawDegrees, 1)})°"
        } ?: "3D unsolved (Full 3D off?)"
        return "${tag?.meaning ?: "NOT A BIOBUZZ TAG"}$family | tx ${fmt(f.txDegrees, 1)}° ty ${fmt(f.tyDegrees, 1)}° " +
            "ta ${fmt(f.areaPercent, 2)} | $pose"
    }

    // --------------------------------------------------------------- Ball camera

    fun ballCameraSections(bag: TelemetryBag, camera: BallCameraSubsystem, detailed: Boolean) {
        val obs = camera.observation
        val frame = obs.frame
        val tracker = camera.tracker
        val lens = camera.lensReport
        bag.section("Ball Camera") {
            put("state", camera.health())
            put("stream", "${camera.runningStream} (requested ${camera.requestedStream})")
            put("lens calibration", lens.describe())
            put("frame", "#${frame?.frameNumber ?: 0} ${obs.frameStatus}; settings v${frame?.settingsVersion ?: 0}/${camera.settingsVersion}")
            put(
                "age ms",
                if (obs.captureTimeValid) "${fmt(obs.ageMs, 1)} since SDK capture time" else "${fmt(obs.ageMs, 1)} since publish (capture time unavailable)",
            )
            put("capture→publish ms", obs.captureToPublishMs, decimals = 1)
            put("processing ms", frame?.processingMs ?: Double.NaN, decimals = 1)
            put("publish→robot ms", obs.publishToReceiptMs, decimals = 1)
            put("processed fps (detector)", tracker.processedFps, decimals = 1)
            put("received fps (robot)", tracker.receivedFps, decimals = 1)
            put("library fps (delivery)", camera.libraryReportedFps, decimals = 1)
            put("skipped / repeat ticks", "${tracker.skippedFrames} / ${tracker.repeatTicks}")
            put("processor faults", "${camera.processorFaults}${camera.lastProcessorError?.let { " ($it)" } ?: ""}")
        }
        bag.section("Ball Target") {
            put("status", obs.targetStatus)
            put("coords", "image px of ${frame?.widthPx ?: 0}x${frame?.heightPx ?: 0}, origin top-left, +x right, +y down")
            val selected = obs.selected
            if (selected == null) {
                put("selected", "none")
            } else {
                val b = selected.blob
                put("center px", "(${fmt(b.centroidXPx, 1)}, ${fmt(b.centroidYPx, 1)})")
                put("radius px / area px²", "${fmt(b.enclosingRadiusPx, 1)} / ${fmt(b.contourAreaPx, 0)}")
                put("circ / aspect / density", "${fmt(b.circularity, 2)} / ${fmt(b.aspectRatio, 2)} / ${fmt(b.density, 2)}")
                val angles = lens.intrinsics?.rayAnglesDegrees(b.centroidXPx, b.centroidYPx)
                put(
                    "ray angle deg (lens only)",
                    angles?.let { "h ${fmt(it.first, 2)} (+right), v ${fmt(it.second, 2)} (+down)" } ?: "unavailable: no lens calibration",
                )
            }
        }
        if (detailed) {
            bag.section("Ball Candidates") {
                put("contours / measured / accepted", "${frame?.contourCount ?: 0} / ${frame?.selection?.evaluatedCount ?: 0} / ${frame?.selection?.acceptedCount ?: 0}")
                obs.candidates.forEachIndexed { i, c ->
                    val b = c.blob
                    val mark = when {
                        c === obs.selected -> "SELECTED"
                        c.accepted -> "ok"
                        else -> "rejected ${c.rejection?.label}"
                    }
                    put(
                        "#$i",
                        "$mark (${fmt(b.centroidXPx, 0)},${fmt(b.centroidYPx, 0)}) r${fmt(b.enclosingRadiusPx, 0)} " +
                            "a${fmt(b.contourAreaPx, 0)} c${fmt(b.circularity, 2)} ar${fmt(b.aspectRatio, 1)} d${fmt(b.density, 2)}",
                    )
                }
            }
            cameraControlSection(bag, camera)
            previewSection(bag)
        }
    }

    private fun cameraControlSection(bag: TelemetryBag, camera: BallCameraSubsystem) {
        val status = camera.controlStatus
        val caps = status.capabilities
        val req = status.requested
        val rb = status.readback
        bag.section("Camera Controls") {
            put("worker", "${status.state}; ${status.deviceWrites} device writes${status.lastError?.let { "; error $it" } ?: ""}")
            if (caps == null) {
                put("probe", "not yet (waits for STREAMING)")
            } else {
                put(
                    "exposure",
                    if (caps.exposureSupported) "manual=${caps.manualExposureSupported} auto=${caps.autoExposureMode} " +
                        "${caps.minExposureMicros}–${caps.maxExposureMicros} µs" else "unsupported",
                )
                put("gain", if (caps.gainSupported) "${caps.minGain}–${caps.maxGain}" else "unsupported")
                put("white balance", if (caps.whiteBalanceSupported) "${caps.minWhiteBalanceKelvin}–${caps.maxWhiteBalanceKelvin} K" else "unsupported")
                put("focus (report only)", "length=${caps.focusLengthSupported} modes=${caps.focusModes}")
                if (caps.probeNotes.isNotEmpty()) put("probe notes", caps.probeNotes.joinToString("; "))
            }
            if (req != null) {
                put(
                    "exposure req → device",
                    "${if (req.exposureManual) "MANUAL ${req.exposureMicros} µs" else "AUTO"} → ${rb?.exposureMode ?: "?"} ${rb?.exposureMicros ?: "?"} µs",
                )
                put("gain req → device", "${if (req.gain < 0) "untouched" else req.gain} → ${rb?.gain ?: "?"}")
                put(
                    "WB req → device",
                    "${if (req.whiteBalanceManual) "MANUAL ${req.whiteBalanceKelvin} K" else "AUTO"} → ${rb?.whiteBalanceMode ?: "?"} ${rb?.whiteBalanceKelvin ?: "?"} K",
                )
            }
            if (status.notes.isNotEmpty()) put("apply notes", status.notes.joinToString("; "))
        }
    }

    private fun previewSection(bag: TelemetryBag) {
        bag.section("Preview") {
            put("mode", "${BallVisionSettings.previewModeOf(BallVisionConfig.previewMode)} (enabled=${BallVisionConfig.previewEnabled})")
            put("view", "Control Hub HDMI or scrcpy: Robot Controller screen")
            put("change", "X cycles overlay/original/mask, B toggles rendering (after START); or Panels ballVision.previewMode 0/1/2")
        }
    }

    fun cyclePreviewMode() {
        val next = (BallVisionSettings.previewModeOf(BallVisionConfig.previewMode).ordinal + 1) % BallPreviewMode.entries.size
        BallVisionConfig.previewMode = next
    }

    fun togglePreview() {
        BallVisionConfig.previewEnabled = !BallVisionConfig.previewEnabled
    }

    // -------------------------------------------------------------------- Timing

    fun timingSection(bag: TelemetryBag, robot: Robot, timing: LoopTimingStats, startup: Startup) {
        bag.section("Timing Comparison") {
            put("cameras open", if (startup.runBothCameras) "BOTH" else "one")
            put("loop ms now / run mean / run max", "${fmt(robot.lastLoopNanos / 1e6, 1)} / ${fmt(timing.meanMs, 1)} / ${fmt(timing.maxMs, 1)}")
            put("periodic ms now", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 2)
            put("loops", timing.count)
        }
    }

    // ---------------------------------------------------------------- Lab record

    fun measurements(
        robot: Robot,
        timing: LoopTimingStats,
        startup: Startup,
        limelight: LimelightSubsystem?,
        sightings: TagSightingTracker?,
        camera: BallCameraSubsystem?,
    ): List<Pair<String, String>> {
        val m = ArrayList<Pair<String, String>>()
        m += "camerasOpen" to if (startup.runBothCameras) "both" else "one"
        m += "loop.count" to timing.count.toString()
        m += "loop.meanMs" to fmt(timing.meanMs, 2)
        m += "loop.maxMs" to fmt(timing.maxMs, 2)
        m += "loop.note" to "percentiles: make debug on this run's WPILOG"
        if (limelight != null) {
            m += "limelight.pipeline" to "${limelight.activePipelineIndex} ${limelight.pipelineType} (expected ${limelight.pipelineIndex})"
            m += "limelight.newFramesHz" to fmt(limelight.resultRateHz, 1)
            m += "limelight.receiptAgeMs" to limelight.resultAgeMs.toString()
            m += "limelight.frameAgeMs" to fmt(limelight.frameAgeMs, 1)
            m += "limelight.estimatedCaptureAgeMs" to fmt(limelight.estimatedCaptureAgeMs, 1)
            m += "limelight.staleTicks" to limelight.staleTickCount.toString()
        }
        sightings?.all()?.forEach { s ->
            m += "tag.${s.id}" to "${s.catalogTag?.meaning ?: "unknown"}; ${s.framesSeen}/${sightings.framesObserved} frames; " +
                "last tx ${fmt(s.latest.txDegrees, 2)} ty ${fmt(s.latest.tyDegrees, 2)}; " +
                (s.latest.targetPoseCameraSpace?.let { "|d| ${fmt(it.distanceMeters, 3)} m" } ?: "3D unsolved")
        }
        if (camera != null) {
            val obs = camera.observation
            val lens = camera.lensReport
            m += "camera.stream" to camera.runningStream.toString()
            m += "camera.state" to camera.cameraState
            m += "camera.processedFps" to fmt(camera.tracker.processedFps, 1)
            m += "camera.receivedFps" to fmt(camera.tracker.receivedFps, 1)
            m += "camera.libraryFps" to fmt(camera.libraryReportedFps, 1)
            m += "camera.frameAgeMs" to fmt(obs.ageMs, 1)
            m += "camera.captureTimeValid" to obs.captureTimeValid.toString()
            m += "camera.processingMs" to fmt(obs.frame?.processingMs ?: Double.NaN, 1)
            m += "camera.skippedFrames" to camera.tracker.skippedFrames.toString()
            m += "camera.processorFaults" to camera.processorFaults.toString()
            m += "camera.lens" to (lens.describe() + (lens.intrinsics?.let { " fx=${it.fx} fy=${it.fy} cx=${it.cx} cy=${it.cy} k=${it.distortion}" } ?: ""))
            m += "camera.target" to (obs.selected?.blob?.let {
                "${obs.targetStatus} (${fmt(it.centroidXPx, 1)},${fmt(it.centroidYPx, 1)}) r=${fmt(it.enclosingRadiusPx, 1)} circ=${fmt(it.circularity, 2)}"
            } ?: obs.targetStatus.name)
            val status = camera.controlStatus
            m += "controls.capabilities" to status.capabilities.toString()
            m += "controls.requested" to status.requested.toString()
            m += "controls.readback" to status.readback.toString()
            m += "controls.notes" to status.notes.joinToString("; ")
        }
        return m
    }

    fun fmt(value: Double, decimals: Int): String =
        if (value.isFinite()) "%.${decimals}f".format(Locale.US, value) else value.toString()
}

/** Whole-run loop-time mean and maximum, for comparing one camera against both. */
internal class LoopTimingStats {
    var count = 0L
        private set
    private var totalNanos = 0L
    private var maxNanos = 0L

    val meanMs: Double get() = if (count == 0L) Double.NaN else totalNanos / count / 1e6
    val maxMs: Double get() = maxNanos / 1e6

    fun record(loopNanos: Long) {
        if (loopNanos <= 0) return
        count++
        totalNanos += loopNanos
        if (loopNanos > maxNanos) maxNanos = loopNanos
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
) : SubsystemBase("VisionLabRecord") {

    private var reportedWritten = 0
    private var reportedError: String? = null

    fun save(measurements: List<Pair<String, String>>) {
        val now = System.currentTimeMillis()
        val contents = VisionLabRecord.build(
            opModeName = opModeName,
            wallClockMs = now,
            configSchema = RobotConfig.CONFIG_SCHEMA,
            sections = listOf(
                VisionLabRecord.sectionFromStore("ballVision", BallVisionConfig.compiledDefaults()),
                VisionLabRecord.sectionFromStore("visionDiagnostics", VisionDiagnosticsConfig.compiledDefaults()),
            ),
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
