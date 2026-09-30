package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.LoopPhase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightFiducial
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.apriltags.TagSightingTracker
import org.firstinspires.ftc.teamcode.vision.ball.BallCameraSubsystem
import org.firstinspires.ftc.teamcode.vision.ball.BallCameraConfig
import org.firstinspires.ftc.teamcode.vision.diagnostics.VisionDiagnosticsConfig

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
        ConfigStore.register("ballVision", BallCameraConfig, BallCameraConfig::resetDefaults)
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
        if (camera != null && camera.restartRequired) pending += "ballVision init settings (color, region, blur/erode/dilate, exposure)"
        if (pending.isEmpty()) return
        bag.section("RESTART REQUIRED") {
            put("changed", pending.joinToString("; "))
            put("running", "both cameras=${startup.runBothCameras}, LL pipeline ${startup.tagPipelineIndex}")
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
                put("${cell.stickerLabel} ($ids)", last)
            }
            put("note", "tags identify a CELL; Hive Tag Survey infers which CELL is raised")
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
        bag.section("Ball Camera") {
            put("state", camera.health())
            put("lens calibration", camera.lensReport.describe())
            put("exposure", camera.exposureStatus)
            put("frame age ms (robot clock)", camera.frameAgeMs, decimals = 1)
            put("library fps (delivery)", camera.libraryReportedFps, decimals = 1)
        }
        bag.section("Ball Target") {
            put("coords", "image px of ${BallCameraSubsystem.WIDTH_PX}x${BallCameraSubsystem.HEIGHT_PX}, origin top-left, +x right, +y down")
            val t = camera.target
            if (t == null) {
                put("target", "none")
            } else {
                put("center px", "(${fmt(t.xPx, 1)}, ${fmt(t.yPx, 1)})")
                put("radius px / area px²", "${fmt(t.radiusPx, 1)} / ${fmt(t.areaPx, 0)}")
                put("circ / aspect / density", "${fmt(t.circularity, 2)} / ${fmt(t.aspectRatio, 2)} / ${fmt(t.density, 2)}")
                put(
                    "ray angle deg (lens only)",
                    if (t.horizontalDeg.isFinite()) "h ${fmt(t.horizontalDeg, 2)} (+right), v ${fmt(t.verticalDeg, 2)} (+down)" else "unavailable: no lens calibration",
                )
            }
        }
        if (detailed) {
            bag.section("Ball Candidates") {
                put("passed filters", camera.candidates.size)
                camera.candidates.forEachIndexed { i, c ->
                    put(
                        "#$i",
                        "${if (i == 0) "TARGET" else "ok"} (${fmt(c.xPx, 0)},${fmt(c.yPx, 0)}) r${fmt(c.radiusPx, 0)} " +
                            "a${fmt(c.areaPx, 0)} c${fmt(c.circularity, 2)} ar${fmt(c.aspectRatio, 1)} d${fmt(c.density, 2)}",
                    )
                }
            }
            bag.section("Preview") {
                put("enabled", BallCameraConfig.previewEnabled)
                put("view", "Control Hub HDMI or scrcpy: Robot Controller screen; blobs that passed are outlined")
                put("change", "B toggles rendering (after START), or Panels ballVision.previewEnabled")
            }
        }
    }

    fun togglePreview() {
        BallCameraConfig.previewEnabled = !BallCameraConfig.previewEnabled
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
            val lens = camera.lensReport
            m += "camera.state" to camera.cameraState
            m += "camera.libraryFps" to fmt(camera.libraryReportedFps, 1)
            m += "camera.frameAgeMs" to fmt(camera.frameAgeMs, 1)
            m += "camera.exposure" to camera.exposureStatus
            m += "camera.lens" to (lens.describe() + (lens.intrinsics?.let { " fx=${it.fx} fy=${it.fy} cx=${it.cx} cy=${it.cy} k=${it.distortion}" } ?: ""))
            m += "camera.target" to (camera.target?.let {
                "(${fmt(it.xPx, 1)},${fmt(it.yPx, 1)}) r=${fmt(it.radiusPx, 1)} circ=${fmt(it.circularity, 2)}"
            } ?: "none")
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
