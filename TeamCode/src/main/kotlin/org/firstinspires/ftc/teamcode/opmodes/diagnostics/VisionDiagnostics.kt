package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.core.LoopPhase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.subsystems.LimelightFiducial
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.vision.BallCameraConfig
import org.firstinspires.ftc.teamcode.vision.BallCameraSubsystem
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags

/** Wiring and telemetry shared by the Limelight AprilTag Test and Ball Tracking Test. */
internal object VisionDiagnostics {

    val limelightRequirement = Preflight.Requirement(LimelightSubsystem.DEFAULT_HARDWARE_NAME, Limelight3A::class.java)
    val ballCameraRequirement = Preflight.Requirement(BallCameraSubsystem.DEFAULT_HARDWARE_NAME, WebcamName::class.java)

    /** [VisionDiagnosticsConfig], read once in `configure()`. */
    data class Startup(
        val runBothCameras: Boolean,
        val tagPipelineIndex: Int,
        val pollRateHz: Int,
        val maxResultAgeMs: Long,
    ) {
        companion object {
            fun fromConfig() = Startup(
                runBothCameras = VisionDiagnosticsConfig.runBothCameras,
                tagPipelineIndex = VisionDiagnosticsConfig.limelightTagPipelineIndex,
                pollRateHz = VisionDiagnosticsConfig.limelightPollRateHz,
                maxResultAgeMs = VisionDiagnosticsConfig.limelightMaxResultAgeMs.toLong(),
            )
        }
    }

    fun limelight(startup: Startup) = LimelightSubsystem(
        pipelineIndex = startup.tagPipelineIndex,
        pollRateHz = startup.pollRateHz,
        maxResultAgeMs = startup.maxResultAgeMs,
    )

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
                put("change", "Circle toggles rendering (after START), or Panels ballVision.previewEnabled")
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
 * Settings shared by the vision diagnostics. Every field decides what hardware
 * the OpMode opens, so it is read once at init and is not in Panels: change it
 * here and hot reload.
 */
object VisionDiagnosticsConfig {
    /** Open the Limelight and the USB camera together to compare timing under combined load. */
    @JvmField var runBothCameras = false

    /** Limelight pipeline slot configured for AprilTags in the web interface. */
    @JvmField var limelightTagPipelineIndex = 0

    @JvmField var limelightPollRateHz = 100

    /** Frames older than this since their first receipt are stale, even if duplicate polls keep arriving. */
    @JvmField var limelightMaxResultAgeMs = 100
}

/**
 * Per-tag sighting history for the AprilTag diagnostic: when each ID was last
 * in a fresh result and how many fresh frames contained it.
 *
 * Feed it only the fiducials of a fresh, matching-pipeline result, once per new
 * device frame ([LimelightSubsystem]
 * already clears them otherwise). Times are robot-loop nanoTime at the tick the
 * frame was first seen — receipt time, not acquisition time.
 */
class TagSightingTracker {

    data class Sighting(
        val id: Int,
        val catalogTag: BiobuzzAprilTags.Tag?,
        val lastSeenNs: Long,
        val framesSeen: Long,
        val latest: LimelightFiducial,
    ) {
        fun ageMs(nowNs: Long): Double = (nowNs - lastSeenNs) / 1e6
    }

    private val sightings = LinkedHashMap<Int, Sighting>()

    var framesObserved: Long = 0
        private set

    fun recordFrame(fiducials: List<LimelightFiducial>, nowNs: Long) {
        framesObserved++
        for (fiducial in fiducials) {
            val previous = sightings[fiducial.id]
            sightings[fiducial.id] = Sighting(
                id = fiducial.id,
                catalogTag = BiobuzzAprilTags.lookup(fiducial.id),
                lastSeenNs = nowNs,
                framesSeen = (previous?.framesSeen ?: 0L) + 1,
                latest = fiducial,
            )
        }
    }

    fun sighting(id: Int): Sighting? = sightings[id]

    /** Every ID ever seen this run, season catalog first then unknown IDs, ascending. */
    fun all(): List<Sighting> = sightings.values.sortedWith(
        compareBy<Sighting> { it.catalogTag == null }.thenBy { it.id },
    )

    /** Newest sighting of any tag on [cell], or null if none has been seen this run. */
    fun latestFor(cell: BiobuzzAprilTags.Cell): Sighting? =
        sightings.values.filter { it.catalogTag?.cell == cell }.maxByOrNull { it.lastSeenNs }

    fun reset() {
        sightings.clear()
        framesObserved = 0
    }
}
