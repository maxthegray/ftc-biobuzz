package org.firstinspires.ftc.teamcode.vision.ball

import android.graphics.Canvas
import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.TimeUnit
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.Clock
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.SettingsChangeLog
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobCriteria
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor.BlobFilter
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Mat
import org.opencv.core.Scalar

/** One blob that passed the filters. Pixels of the full frame, origin top-left; angles NaN without a lens calibration. */
data class BallSighting(
    val xPx: Double,
    val yPx: Double,
    val radiusPx: Double,
    val areaPx: Double,
    val circularity: Double,
    val aspectRatio: Double,
    val density: Double,
    /** Right of the optical axis is positive. */
    val horizontalDeg: Double,
    /** Below the optical axis is positive. */
    val verticalDeg: Double,
)

/**
 * The **init** fields of [BallCameraConfig], validated. Built into the processor
 * at init; a change shows up as [BallCameraSubsystem.restartRequired].
 */
data class DetectionSetup(
    val hsv: Boolean,
    val min: List<Int>,
    val max: List<Int>,
    /** Pixel rectangle `[left, top, right, bottom]`; the whole frame when the configured one is invalid. */
    val roiPx: List<Int>,
    val blurPx: Int,
    val erodePx: Int,
    val dilatePx: Int,
    val closing: Boolean,
    val externalOnly: Boolean,
    val exposureManual: Boolean,
    val exposureMicros: Long,
) {
    companion object {
        fun fromConfig(widthPx: Int, heightPx: Int): DetectionSetup = with(BallCameraConfig) {
            val hsv = colorSpace == BallCameraConfig.COLOR_SPACE_HSV
            val channelMax = listOf(if (hsv) 180 else 255, 255, 255)
            val lows = listOf(channel0Min, channel1Min, channel2Min)
            val highs = listOf(channel0Max, channel1Max, channel2Max)
            val roiValid = roiLeft in 0.0..1.0 && roiRight in 0.0..1.0 && roiTop in 0.0..1.0 && roiBottom in 0.0..1.0 &&
                roiLeft < roiRight && roiTop < roiBottom
            DetectionSetup(
                hsv = hsv,
                min = lows.mapIndexed { i, v -> v.coerceIn(0, channelMax[i]) },
                max = highs.mapIndexed { i, v -> v.coerceIn(0, channelMax[i]) },
                roiPx = if (roiValid) {
                    listOf((roiLeft * widthPx).toInt(), (roiTop * heightPx).toInt(), (roiRight * widthPx).toInt(), (roiBottom * heightPx).toInt())
                } else {
                    listOf(0, 0, widthPx, heightPx)
                },
                blurPx = blurKernelPx.coerceAtLeast(0),
                erodePx = erodeKernelPx.coerceAtLeast(0),
                dilatePx = dilateKernelPx.coerceAtLeast(0),
                closing = morphClosing,
                externalOnly = externalContoursOnly,
                exposureManual = exposureManual,
                exposureMicros = exposureMicros.toLong().coerceAtLeast(1),
            )
        }
    }
}

/** The live shape filters of [BallCameraConfig] as SDK filters, for a frame of [frameAreaPx] pixels. */
fun blobFilters(frameAreaPx: Double): List<BlobFilter> = with(BallCameraConfig) {
    listOf(
        BlobFilter(BlobCriteria.BY_CONTOUR_AREA, minAreaPercent / 100.0 * frameAreaPx, maxAreaPercent / 100.0 * frameAreaPx),
        BlobFilter(BlobCriteria.BY_CIRCULARITY, minCircularity, Double.MAX_VALUE),
        BlobFilter(BlobCriteria.BY_ASPECT_RATIO, 0.0, maxAspectRatio),
        BlobFilter(BlobCriteria.BY_DENSITY, minDensity, Double.MAX_VALUE),
    )
}

/**
 * USB ball camera on the SDK's ColorBlobLocatorProcessor. Read-only; commands
 * nothing. [candidates] holds every blob that passed the filters in the latest
 * frame, largest first, and [target] is the largest. Both clear when no new
 * frame has arrived for `maxObservationAgeMs`.
 *
 * Settings come from [BallCameraConfig] in Panels. The color range, search
 * region, blur/erode/dilate and exposure are built in at init; the shape
 * filters apply live (the SDK guards its filter list).
 */
class BallCameraSubsystem(
    val hardwareName: String = DEFAULT_HARDWARE_NAME,
    private val clock: Clock = Clock.SYSTEM,
    /** Flight-log event sink, normally `robot::recordEvent`; receives the tuning values in force. */
    eventSink: (String) -> Unit = {},
) : SubsystemBase("BallCamera") {

    private val settingsLog = SettingsChangeLog("ballVision", clock, eventSink)
    private var portal: VisionPortal? = null
    private var processor: ColorBlobLocatorProcessor? = null
    private val lensProbe = LensProbe()
    private var appliedFilters: List<Pair<Double, Double>>? = null
    private var appliedPreview: Boolean? = null
    private var loggedTuning: List<Any>? = null
    private var lastBlobs: List<ColorBlobLocatorProcessor.Blob>? = null
    private var lastFrameNanos = Long.MIN_VALUE

    /** The **init** settings the processor was built with. */
    var setup: DetectionSetup? = null
        private set
    var exposureStatus = "waiting for STREAMING"
        private set
    var candidates: List<BallSighting> = emptyList()
        private set
    val target: BallSighting? get() = candidates.firstOrNull()

    /** True on the tick a new camera frame arrived. */
    var newFrame = false
        private set

    /** Time since the latest frame arrived, on the robot clock; NaN before the first. */
    var frameAgeMs = Double.NaN
        private set

    val restartRequired: Boolean get() = setup != null && setup != DetectionSetup.fromConfig(WIDTH_PX, HEIGHT_PX)
    val cameraState: String get() = portal?.cameraState?.name ?: "NOT_OPENED"
    val libraryReportedFps: Double get() = portal?.fps?.toDouble() ?: Double.NaN
    val lensReport: LensReport get() = lensProbe.report

    override fun init(hardwareMap: HardwareMap) {
        val webcam = hardwareMap.tryGet(WebcamName::class.java, hardwareName)
            ?: throw HardwareConfigError("Missing Webcam named \"$hardwareName\" in active configuration.")
        val s = DetectionSetup.fromConfig(WIDTH_PX, HEIGHT_PX)
        setup = s
        val built = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(
                ColorRange(
                    if (s.hsv) ColorSpace.HSV else ColorSpace.YCrCb,
                    Scalar(s.min[0].toDouble(), s.min[1].toDouble(), s.min[2].toDouble()),
                    Scalar(s.max[0].toDouble(), s.max[1].toDouble(), s.max[2].toDouble()),
                ),
            )
            .setRoi(ImageRegion.asImageCoordinates(s.roiPx[0], s.roiPx[1], s.roiPx[2], s.roiPx[3]))
            .setContourMode(
                if (s.externalOnly) ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY else ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY,
            )
            .setBlurSize(s.blurPx)
            .setMorphOperationType(
                if (s.closing) ColorBlobLocatorProcessor.MorphOperationType.CLOSING else ColorBlobLocatorProcessor.MorphOperationType.OPENING,
            )
            .setErodeSize(s.erodePx)
            .setDilateSize(s.dilatePx)
            .setDrawContours(true)
            .build()
        processor = built
        applyFilters(built)
        settingsLog.update(SettingsChangeLog.valuesOf(BallCameraConfig), "init")
        portal = VisionPortal.Builder()
            .setCamera(webcam)
            .setCameraResolution(Size(WIDTH_PX, HEIGHT_PX))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            .addProcessors(built, lensProbe)
            .build()
    }

    override fun periodic() {
        val active = portal ?: return
        val blobProcessor = processor ?: return
        if (BallCameraConfig.resetToDefaults) BallCameraConfig.resetDefaults()
        logTuningChanges()
        applyFilters(blobProcessor)
        if (appliedPreview != BallCameraConfig.previewEnabled) {
            appliedPreview = BallCameraConfig.previewEnabled
            if (BallCameraConfig.previewEnabled) active.resumeLiveView() else active.stopLiveView()
        }
        if (exposureStatus == "waiting for STREAMING" && active.cameraState == VisionPortal.CameraState.STREAMING) {
            setExposure(active)
        }

        val now = clock.nanos()
        val latest = blobProcessor.blobs
        newFrame = latest != null && latest !== lastBlobs
        if (newFrame) {
            lastBlobs = latest
            lastFrameNanos = now
            val lens = lensReport.intrinsics
            candidates = latest.map { blob ->
                val circle = blob.circle
                val x = circle.x.toDouble()
                val y = circle.y.toDouble()
                val angles = lens?.rayAnglesDegrees(x, y)
                BallSighting(
                    xPx = x,
                    yPx = y,
                    radiusPx = circle.radius.toDouble(),
                    areaPx = blob.contourArea.toDouble(),
                    circularity = blob.circularity,
                    aspectRatio = blob.aspectRatio,
                    density = blob.density,
                    horizontalDeg = angles?.first ?: Double.NaN,
                    verticalDeg = angles?.second ?: Double.NaN,
                )
            }
        }
        frameAgeMs = if (lastFrameNanos == Long.MIN_VALUE) Double.NaN else (now - lastFrameNanos) / 1e6
        if (!newFrame && !(frameAgeMs <= BallCameraConfig.maxObservationAgeMs)) candidates = emptyList()
    }

    /** Reflection only when a value changed or a merged edit is waiting to be written. */
    private fun logTuningChanges() {
        val tuning = with(BallCameraConfig) {
            listOf(
                DetectionSetup.fromConfig(WIDTH_PX, HEIGHT_PX), minAreaPercent, maxAreaPercent, minCircularity, maxAspectRatio,
                minDensity, maxObservationAgeMs, previewEnabled, mountHeightIn, mountPitchDownDeg, mountForwardIn,
                mountLeftIn, mountYawDeg, mountMeasured,
            )
        }
        if (tuning == loggedTuning && !settingsLog.hasPending) return
        loggedTuning = tuning
        settingsLog.update(SettingsChangeLog.valuesOf(BallCameraConfig), "live")
    }

    private fun applyFilters(target: ColorBlobLocatorProcessor) {
        val filters = blobFilters((WIDTH_PX * HEIGHT_PX).toDouble())
        val key = filters.map { it.minValue to it.maxValue }
        if (key == appliedFilters) return
        appliedFilters = key
        target.removeAllFilters()
        filters.forEach(target::addFilter)
    }

    /** Runs once, when the camera starts streaming: USB camera controls block until the device answers. */
    private fun setExposure(active: VisionPortal) {
        val s = setup ?: return
        exposureStatus = try {
            val exposure = active.getCameraControl(ExposureControl::class.java)
            if (s.exposureManual) {
                exposure.mode = ExposureControl.Mode.Manual
                exposure.setExposure(s.exposureMicros, TimeUnit.MICROSECONDS)
                "manual ${exposure.getExposure(TimeUnit.MICROSECONDS)} µs (requested ${s.exposureMicros})"
            } else {
                exposure.mode = ExposureControl.Mode.ContinuousAuto
                "auto"
            }
        } catch (e: RuntimeException) {
            "failed: ${e.message}"
        }
    }

    override fun health(): String = when {
        portal == null -> "not opened"
        restartRequired -> "restart required for ballVision init settings"
        else -> "${cameraState.lowercase()}; ${candidates.size} ball(s)"
    }

    override fun logState(log: StateLog) {
        log.put("camera/state", cameraState)
        log.put("camera/libraryReportedFps", libraryReportedFps)
        log.put("camera/restartRequired", restartRequired)
        log.put("frame/new", newFrame)
        log.put("frame/ageMs", frameAgeMs)
        log.put("frame/widthPx", WIDTH_PX.toLong())
        log.put("frame/heightPx", HEIGHT_PX.toLong())
        log.put("candidates/xPx", DoubleArray(candidates.size) { candidates[it].xPx })
        log.put("candidates/yPx", DoubleArray(candidates.size) { candidates[it].yPx })
        log.put("candidates/radiusPx", DoubleArray(candidates.size) { candidates[it].radiusPx })
        log.put("candidates/areaPx", DoubleArray(candidates.size) { candidates[it].areaPx })
        log.put("candidates/circularity", DoubleArray(candidates.size) { candidates[it].circularity })
        log.put("candidates/horizontalDeg", DoubleArray(candidates.size) { candidates[it].horizontalDeg })
        log.put("candidates/verticalDeg", DoubleArray(candidates.size) { candidates[it].verticalDeg })
        log.put("candidates/selectedIndex", if (candidates.isEmpty()) -1L else 0L)
        val t = target
        log.put("target/xPx", t?.xPx ?: Double.NaN)
        log.put("target/yPx", t?.yPx ?: Double.NaN)
        log.put("target/radiusPx", t?.radiusPx ?: Double.NaN)
        log.put("target/areaPx", t?.areaPx ?: Double.NaN)
        log.put("target/circularity", t?.circularity ?: Double.NaN)
        log.put("target/horizontalDeg", t?.horizontalDeg ?: Double.NaN)
        log.put("target/verticalDeg", t?.verticalDeg ?: Double.NaN)
        log.put("mount/measured", BallCameraConfig.mountMeasured)
        log.put("mount/heightIn", BallCameraConfig.mountHeightIn)
        log.put("mount/pitchDownDeg", BallCameraConfig.mountPitchDownDeg)
        log.put("mount/forwardIn", BallCameraConfig.mountForwardIn)
        log.put("mount/leftIn", BallCameraConfig.mountLeftIn)
        log.put("mount/yawDeg", BallCameraConfig.mountYawDeg)
    }

    /**
     * `VisionPortal.close()` can wait on the SDK's camera-state lock while the
     * device is still opening, so it runs on its own thread; EasyOpenCV also
     * closes the camera when the OpMode ends.
     */
    override fun stop() {
        val active = portal ?: return
        portal = null
        Thread({ runCatching { active.close() } }, "ball-camera-close").apply { isDaemon = true }.start()
    }

    /** Captures the SDK's lens calibration, which reaches a processor only through [init]. */
    private class LensProbe : VisionProcessor {
        @Volatile var report: LensReport = LensReport.NOT_YET_KNOWN

        override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
            report = LensReport.of(width, height, calibration)
        }

        override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? = null

        override fun onDrawFrame(
            canvas: Canvas,
            onscreenWidth: Int,
            onscreenHeight: Int,
            scaleBmpPxToCanvasPx: Float,
            scaleCanvasDensity: Float,
            userContext: Any?,
        ) {}
    }

    companion object {
        const val DEFAULT_HARDWARE_NAME = "ballCamera"
        const val WIDTH_PX = 640
        const val HEIGHT_PX = 480
    }
}
