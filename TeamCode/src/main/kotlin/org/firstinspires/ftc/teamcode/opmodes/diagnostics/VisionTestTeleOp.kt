package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale
import java.util.TreeMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.LoopPhase
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.SettingsChangeLog
import org.firstinspires.ftc.teamcode.subsystems.LimelightFiducial
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.vision.BallCameraConfig
import org.firstinspires.ftc.teamcode.vision.BallCameraSubsystem
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Alliance
import org.firstinspires.ftc.teamcode.vision.BiobuzzAprilTags.Cell
import org.firstinspires.ftc.teamcode.vision.CellGoal
import org.firstinspires.ftc.teamcode.vision.GoalGeometry
import org.firstinspires.ftc.teamcode.vision.HiveConfig
import org.firstinspires.ftc.teamcode.vision.HiveTracker
import org.firstinspires.ftc.teamcode.vision.TagRow
import org.firstinspires.ftc.teamcode.vision.Vec3

/**
 * Stationary camera diagnostic. Commands no motors, applies no localization
 * corrections. Opens every camera in the active configuration (see
 * [VisionTestConfig]) and shows one page at a time; dpad left/right changes
 * page, in INIT too.
 *
 *  - **Ball camera**: yellow-pollen detection through the SDK's
 *    ColorBlobLocatorProcessor, tuned from Panels (`ballVision`), with the
 *    detections outlined on the Robot Controller preview. Circle toggles the
 *    preview.
 *  - **Limelight tags**: every fiducial in each fresh result with its BIOBUZZ
 *    meaning, camera-relative measurements and freshness, plus a per-tag
 *    sighting history. Infers nothing about HIVE state.
 *  - **HIVE survey**: the Limelight on a tripod (turret angle held at 0; the
 *    `hive` camera fields describe the tripod). For every tag of both
 *    alliances: its height, raised/lowered class, the CELL opening centre it
 *    implies and how far that is from its CELL's fused goal, plus each HIVE's
 *    inferred state. Four tags of one CELL agreeing within about an inch
 *    checks the camera mount, FIRST's cluster geometry and the Limelight's
 *    rotation convention together. Circle clears the survey memory.
 *
 * Loop timing covers every open camera; turn one off in [VisionTestConfig]
 * to compare one camera against both.
 */
@TeleOp(name = "Vision Test", group = "Diagnostics")
class VisionTestTeleOp : OpModeBase() {

    private enum class Page(val title: String) {
        BALL("Ball camera"),
        TAGS("Limelight tags"),
        HIVE("HIVE survey"),
    }

    private var limelight: LimelightSubsystem? = null
    private var hive: HiveTracker? = null
    private var ballCamera: BallCameraSubsystem? = null
    private lateinit var pages: List<Page>
    private var page = 0
    private val sightings = TagSightingTracker()
    private val survey = HiveSurveyMemory()
    private val timing = LoopTimingStats()

    override val requiredDevices: List<Preflight.Requirement> get() = emptyList()
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        robot.recordEvent(SettingsChangeLog.describe("visionTest", VisionTestConfig))
        val hasLimelight = hardwareMap.tryGet(Limelight3A::class.java, RobotConfig.Vision.LIMELIGHT) != null
        val hasBallCamera = hardwareMap.tryGet(WebcamName::class.java, RobotConfig.Vision.BALL_CAMERA) != null
        if (VisionTestConfig.openBallCamera && hasBallCamera) {
            ballCamera = robot.register(BallCameraSubsystem(eventSink = robot::recordEvent))
        }
        if (VisionTestConfig.openLimelight && hasLimelight) {
            robot.recordEvent(SettingsChangeLog.describe("hive", HiveConfig))
            val ll = robot.register(
                LimelightSubsystem(
                    pipelineIndex = VisionTestConfig.limelightTagPipelineIndex,
                    pollRateHz = VisionTestConfig.limelightPollRateHz,
                    maxResultAgeMs = VisionTestConfig.limelightMaxResultAgeMs.toLong(),
                ),
            )
            limelight = ll
            hive = robot.register(HiveTracker(ll, { 0.0 }, eventSink = robot::recordEvent, clock = robot.clock))
        }
        pages = buildList {
            if (ballCamera != null) add(Page.BALL)
            if (limelight != null) addAll(listOf(Page.TAGS, Page.HIVE))
        }
        if (pages.isEmpty()) {
            throw HardwareConfigError(
                "No camera to test: need Limelight 3A \"${RobotConfig.Vision.LIMELIGHT}\" or webcam " +
                    "\"${RobotConfig.Vision.BALL_CAMERA}\" in the active configuration " +
                    "(and openLimelight / openBallCamera on in VisionTestConfig)",
            )
        }
    }

    override fun onStart() {
        robot.recordEvent(
            "Vision Test started; limelight=${limelight != null} ballCamera=${ballCamera != null} " +
                "hive mount measured=${HiveConfig.mountMeasured}",
        )
    }

    override fun onInitLoop() = tick()

    override fun onLoop() {
        timing.record(robot.lastLoopNanos)
        tick()
    }

    private fun tick() {
        if (driver.pressed(Button.DPAD_RIGHT)) page = (page + 1) % pages.size
        if (driver.pressed(Button.DPAD_LEFT)) page = (page + pages.size - 1) % pages.size
        val current = pages[page]
        if (driver.pressed(Button.CIRCLE)) {
            when (current) {
                Page.BALL -> BallCameraConfig.previewEnabled = !BallCameraConfig.previewEnabled
                Page.HIVE -> {
                    survey.reset()
                    robot.recordEvent("HIVE survey memory cleared")
                }
                Page.TAGS -> {}
            }
        }

        val now = robot.clock.nanos()
        limelight?.let { ll ->
            if (ll.newFrameThisTick && ll.resultFresh && ll.pipelineMatches) sightings.recordFrame(ll.fiducials, now)
        }
        hive?.let { h -> if (h.newFrameThisTick) h.lastFrameCaptureNanos?.let { survey.record(h, it) } }

        telemetryBag.section("Vision Test") {
            put("page", "${current.title} (${page + 1}/${pages.size}); dpad ←/→ changes page")
            put("cameras open", listOfNotNull(limelight?.let { "Limelight" }, ballCamera?.let { "ball camera" }).joinToString(" + "))
            when (current) {
                Page.BALL -> put("Circle", "toggles the preview")
                Page.HIVE -> put("Circle", "clears the survey memory")
                Page.TAGS -> {}
            }
        }
        when (current) {
            Page.BALL -> ballCameraSections(ballCamera!!)
            Page.TAGS -> tagSections(limelight!!, now)
            Page.HIVE -> hiveSections(limelight!!, hive!!, now)
        }
        telemetryBag.section("Timing") {
            put("loop ms now / run mean / run max", "${fmt(robot.lastLoopNanos / 1e6, 1)} / ${fmt(timing.meanMs, 1)} / ${fmt(timing.maxMs, 1)}")
            put("periodic ms now", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 2)
            put("loops", timing.count)
        }
    }

    // --------------------------------------------------------------- Ball camera

    private fun ballCameraSections(camera: BallCameraSubsystem) {
        telemetryBag.section("Ball Camera") {
            put("state", camera.health())
            put("lens calibration", camera.lensReport.describe())
            put("exposure", camera.exposureStatus)
            put("frame age ms (robot clock)", camera.frameAgeMs, decimals = 1)
            put("library fps (delivery)", camera.libraryReportedFps, decimals = 1)
        }
        telemetryBag.section("Ball Target") {
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
        telemetryBag.section("Ball Candidates") {
            put("passed filters", camera.candidates.size)
            camera.candidates.forEachIndexed { i, c ->
                put(
                    "#$i",
                    "${if (i == 0) "TARGET" else "ok"} (${fmt(c.xPx, 0)},${fmt(c.yPx, 0)}) r${fmt(c.radiusPx, 0)} " +
                        "a${fmt(c.areaPx, 0)} c${fmt(c.circularity, 2)} ar${fmt(c.aspectRatio, 1)} d${fmt(c.density, 2)}",
                )
            }
        }
        telemetryBag.section("Preview") {
            put("enabled", BallCameraConfig.previewEnabled)
            put("view", "Control Hub HDMI or scrcpy: Robot Controller screen; blobs that passed are outlined")
            put("change", "Circle, or Panels ballVision.previewEnabled")
        }
    }

    // ----------------------------------------------------------------- Limelight

    private fun limelightSection(limelight: LimelightSubsystem) {
        telemetryBag.section("Limelight") {
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
    }

    private fun tagSections(limelight: LimelightSubsystem, now: Long) {
        limelightSection(limelight)
        telemetryBag.section("Tags Now") {
            put("frame", "${limelight.fiducials.size} tag(s); fresh=${limelight.resultFresh}")
            for (fiducial in limelight.fiducials) put("ID ${fiducial.id}", describeFiducial(fiducial))
        }
        telemetryBag.section("Tags Seen This Run") {
            val all = sightings.all()
            if (all.isEmpty()) put("none", "no tag in any fresh frame yet")
            for (s in all) {
                val meaning = s.catalogTag?.meaning ?: "NOT A BIOBUZZ TAG"
                put("ID ${s.id}", "$meaning; last ${fmt(s.ageMs(now), 0)} ms ago; ${s.framesSeen}/${sightings.framesObserved} frames")
            }
        }
        telemetryBag.section("HIVE Cells") {
            for (cell in BiobuzzAprilTags.cells) {
                val ids = BiobuzzAprilTags.tagsOf(cell).joinToString(",") { it.id.toString() }
                val last = sightings.latestFor(cell)?.let { "last tag ${fmt(it.ageMs(now), 0)} ms ago" } ?: "no tag seen"
                put("${cell.stickerLabel} ($ids)", last)
            }
            put("note", "tags identify a CELL; the HIVE survey page infers which CELL is raised")
        }
        telemetryBag.section("Procedure") {
            put("1", "Limelight web UI: pipeline ${limelight.pipelineIndex} = AprilTag 36h11, 82.55 mm, Full 3D")
            put("2", "Hold one tag still; ID and meaning must match the sticker")
            put("3", "Move left/right: tx sign; up/down: ty sign; note both")
            put("4", "Tape-measure camera→tag; compare |d|")
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

    // ---------------------------------------------------------------- HIVE survey

    private fun hiveSections(limelight: LimelightSubsystem, hive: HiveTracker, now: Long) {
        limelightSection(limelight)
        telemetryBag.section("Survey Setup") {
            put("mount", if (HiveConfig.mountMeasured) "measured" else "NOT MEASURED: heights and raised/lowered are meaningless")
            put(
                "lens",
                "height ${fmt(HiveConfig.cameraHeightIn, 1)} in, pitch up ${fmt(HiveConfig.cameraPitchUpDeg, 1)}°, " +
                    "yaw ${fmt(HiveConfig.cameraYawDeg, 1)}°, fwd ${fmt(HiveConfig.cameraForwardIn, 1)} / " +
                    "left ${fmt(HiveConfig.cameraLeftIn, 1)} in from the turret axis",
            )
            put("turret", "held at 0° (tripod)")
            put("raised if tag above", "${fmt(HiveConfig.raisedMinHeightIn, 1)} ± ${fmt(HiveConfig.classificationMarginIn, 1)} in")
        }
        telemetryBag.section("HIVEs") {
            for (alliance in Alliance.entries) {
                val state = hive.state(alliance)?.let {
                    "${it.raised.name.lowercase(Locale.US)} CELL raised${if (hive.stateAssumed(alliance)) " (assumed)" else ""}"
                } ?: "unknown"
                val goal = hive.goal(alliance)?.let {
                    "goal ${fmt(it.horizontalDistanceIn, 1)} in at ${fmt(Math.toDegrees(it.turretBearingRad), 1)}°, " +
                        "height ${fmt(it.heightIn, 1)} in, spread ${fmt(it.spreadIn, 2)} in, tags ${it.tagIds.joinToString(",")}"
                } ?: "no goal"
                put(alliance.name, "$state; tips ${hive.tipCount(alliance)}; $goal")
            }
        }
        telemetryBag.section("CELL Goals (latest)") {
            for (cell in BiobuzzAprilTags.cells) {
                val seen = survey.cell(cell)
                put(cell.stickerLabel, seen?.let { describeCell(it.value) + "; ${ago(now, it.captureNanos)}" } ?: "not seen")
            }
        }
        telemetryBag.section("Tags (latest)") {
            for (tag in BiobuzzAprilTags.tags) {
                val seen = survey.tag(tag.id) ?: continue
                put("ID ${tag.id}", describeTag(seen.value) + "; ${ago(now, seen.captureNanos)}")
            }
        }
        telemetryBag.section("Procedure") {
            put("1", "Tripod: measure lens height, pitch up; set hive.camera*, mountMeasured=true")
            put("2", "Limelight pipeline ${limelight.pipelineIndex}: AprilTag 36h11, 82.55 mm, Full 3D")
            put("3", "Each CELL: spread < ~1 in, all four tags agree (else rotation convention)")
            put("4", "Tape-measure tripod → CELL opening centre; compare horizontal")
            put("5", "Tip each HIVE; note tag heights both ways; set hive.raisedMinHeightIn between")
            put("6", "Circle clears the survey memory")
        }
    }
}

/**
 * Settings for the Vision Test. Every field decides what hardware the OpMode
 * opens, so it is read once at init and is not in Panels: change it here and
 * hot reload.
 */
object VisionTestConfig {
    /** Open the Limelight when the configuration has one. */
    @JvmField var openLimelight = true

    /** Open the USB ball camera when the configuration has one. */
    @JvmField var openBallCamera = true

    /** Limelight pipeline slot configured for AprilTags in the web interface. */
    @JvmField var limelightTagPipelineIndex = 0

    @JvmField var limelightPollRateHz = 100

    /** Frames older than this since their first receipt are stale, even if duplicate polls keep arriving. */
    @JvmField var limelightMaxResultAgeMs = 100
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
 * Per-tag sighting history for the tag page: when each ID was last in a fresh
 * result and how many fresh frames contained it.
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
    fun latestFor(cell: Cell): Sighting? =
        sightings.values.filter { it.catalogTag?.cell == cell }.maxByOrNull { it.lastSeenNs }

    fun reset() {
        sightings.clear()
        framesObserved = 0
    }
}

/**
 * The HIVE survey's memory: the latest row of every season tag and the latest
 * fused goal of every CELL seen this run, so telemetry lists all 16 tags even
 * when they were surveyed from different spots. Times are frame capture times.
 */
internal class HiveSurveyMemory {

    data class Seen<T>(val value: T, val captureNanos: Long)

    private val tags = TreeMap<Int, Seen<TagRow>>()
    private val cells = LinkedHashMap<Cell, Seen<CellGoal>>()

    fun record(hive: HiveTracker, captureNanos: Long) {
        for (row in hive.rows) tags[row.tag.id] = Seen(row, captureNanos)
        for ((cell, goal) in hive.cellGoals) cells[cell] = Seen(goal, captureNanos)
    }

    fun tag(id: Int): Seen<TagRow>? = tags[id]

    fun cell(cell: Cell): Seen<CellGoal>? = cells[cell]

    fun reset() {
        tags.clear()
        cells.clear()
    }
}

internal fun describeCell(goal: CellGoal): String =
    "${goal.heightClass} goal ${vec(goal.goalRobot)} in; " +
        "horizontal ${fmt(GoalGeometry.horizontalDistanceFromTurretIn(goal.goalRobot), 1)} in; " +
        "turret bearing ${fmt(Math.toDegrees(GoalGeometry.turretBearingRad(goal.goalRobot)), 1)}°; " +
        "spread ${fmt(goal.spreadIn, 2)} in; tags ${goal.tagIds.joinToString(",")}"

internal fun describeTag(row: TagRow): String =
    "${row.tag.meaning}; ${row.heightClass} at ${fmt(row.tagRobot.z, 1)} in; goal ${vec(row.goalRobot)} in; " +
        "off fused ${fmt(row.deviationIn, 2)} in${if (row.usedInFusion) "" else " (dropped)"}; " +
        "faces ${fmt(row.facingBelowHorizontalDeg, 0)}° below horizontal"

private fun fmt(value: Double, decimals: Int): String =
    if (value.isFinite()) "%.${decimals}f".format(Locale.US, value) else value.toString()

private fun vec(v: Vec3) = "(${fmt(v.x, 1)}, ${fmt(v.y, 1)}, ${fmt(v.z, 1)})"

private fun ago(nowNanos: Long, thenNanos: Long) = "seen ${fmt((nowNanos - thenNanos) / 1e9, 1)} s ago"
