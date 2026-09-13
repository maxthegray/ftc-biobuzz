package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/**
 * Color/shape ball detector running on the VisionPortal camera thread.
 *
 * Follows the SDK `ColorBlobLocatorProcessor` pipeline (color conversion, blur,
 * `inRange` threshold, erode/dilate, `findContours`, circularity/aspect/density
 * metrics) but is a separate processor because the SDK one fixes its color
 * range, ROI, and morphology at build time, publishes blobs without a frame
 * timestamp, computes blob metrics lazily on whichever thread asks, and has no
 * mask view.
 *
 * Threading: settings arrive as immutable [DetectionSettings] through an
 * [AtomicReference] read once per frame; results leave as immutable
 * [BallFrameResult]s. Nothing thrown escapes [processFrame] — EasyOpenCV turns
 * a pipeline exception into an emulated robot E-stop. A processor instance may
 * be attached to exactly one VisionPortal per app process (the SDK keeps a
 * static attachment list), so every OpMode run builds a new one.
 */
class BallDetectionProcessor(initial: DetectionSettings) : VisionProcessor {

    private class Versioned(val settings: DetectionSettings, val version: Long)

    private class DrawContext(val result: BallFrameResult, val mode: BallPreviewMode)

    private val settingsRef = AtomicReference(Versioned(initial, 0))
    private val latestRef = AtomicReference<BallFrameResult?>(null)
    private val lensRef = AtomicReference(LensReport.NOT_YET_KNOWN)
    private val faultCount = AtomicLong(0)

    @Volatile var lastError: String? = null
        private set

    // Camera-thread-only working buffers, reused across frames.
    private val converted = Mat()
    private val mask = Mat()
    private val hierarchy = Mat()
    private var erodeKernel: Mat? = null
    private var erodeKernelSize = 0
    private var dilateKernel: Mat? = null
    private var dilateKernelSize = 0
    private var frameNumber = 0L

    // UI-thread-only paints.
    private val roiPaint = strokePaint(Color.WHITE)
    private val acceptedPaint = strokePaint(Color.CYAN)
    private val rejectedPaint = strokePaint(Color.rgb(255, 80, 80))
    private val selectedPaint = strokePaint(Color.MAGENTA)
    private val textPaint = Paint().apply {
        color = Color.WHITE
        isAntiAlias = true
        setShadowLayer(3f, 0f, 0f, Color.BLACK)
    }

    val latestFrame: BallFrameResult? get() = latestRef.get()
    val lensReport: LensReport get() = lensRef.get()
    val faults: Long get() = faultCount.get()

    fun publishSettings(settings: DetectionSettings, version: Long) {
        settingsRef.set(Versioned(settings, version))
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        lensRef.set(lensReportOf(width, height, calibration))
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val start = System.nanoTime()
        val versioned = settingsRef.get()
        return try {
            process(frame, captureTimeNanos, start, versioned)
        } catch (t: Throwable) {
            faultCount.incrementAndGet()
            val message = "${t.javaClass.simpleName}: ${t.message}"
            lastError = message
            latestRef.set(
                BallFrameResult(
                    frameNumber = ++frameNumber,
                    captureTimeNanos = captureTimeNanos,
                    processingStartNanos = start,
                    publishedNanos = System.nanoTime(),
                    widthPx = frame.cols(),
                    heightPx = frame.rows(),
                    roi = PixelRect(0, 0, frame.cols(), frame.rows()),
                    settingsVersion = versioned.version,
                    contourCount = 0,
                    ignoredSmallCount = 0,
                    selection = CandidateSelection.EMPTY,
                    error = message,
                ),
            )
            null
        }
    }

    private fun process(frame: Mat, captureTimeNanos: Long, start: Long, versioned: Versioned): Any? {
        val s = versioned.settings
        val width = frame.cols()
        val height = frame.rows()
        val roi = s.roi.toPixels(width, height)
        val region = frame.submat(Rect(roi.left, roi.top, roi.width, roi.height))
        val blobs = ArrayList<BlobMeasurement>()
        var contourCount = 0
        var ignoredSmall = 0
        try {
            // VisionPortal delivers RGBA frames; these conversions accept 3 or 4 channels.
            val code = if (s.colorSpace == BallColorSpace.HSV) Imgproc.COLOR_RGB2HSV else Imgproc.COLOR_RGB2YCrCb
            Imgproc.cvtColor(region, converted, code)
            if (s.blurKernelPx > 0) {
                Imgproc.GaussianBlur(converted, converted, Size(s.blurKernelPx.toDouble(), s.blurKernelPx.toDouble()), 0.0)
            }
            Core.inRange(
                converted,
                Scalar(s.lower.c0.toDouble(), s.lower.c1.toDouble(), s.lower.c2.toDouble()),
                Scalar(s.upper.c0.toDouble(), s.upper.c1.toDouble(), s.upper.c2.toDouble()),
                mask,
            )
            if (s.morphClosing) {
                dilate(s.dilateKernelPx)
                erode(s.erodeKernelPx)
            } else {
                erode(s.erodeKernelPx)
                dilate(s.dilateKernelPx)
            }

            val contours = ArrayList<MatOfPoint>()
            val mode = if (s.externalContoursOnly) Imgproc.RETR_EXTERNAL else Imgproc.RETR_LIST
            Imgproc.findContours(mask, contours, hierarchy, mode, Imgproc.CHAIN_APPROX_SIMPLE, Point(roi.left.toDouble(), roi.top.toDouble()))
            contourCount = contours.size

            val frameArea = width.toDouble() * height
            val measureFloor = frameArea * s.filters.minAreaPercent / 100.0 * 0.5
            val sized = contours.map { it to Imgproc.contourArea(it) }
                .sortedByDescending { it.second }
            for ((index, pair) in sized.withIndex()) {
                val (contour, area) = pair
                if (area < measureFloor || index >= MAX_MEASURED_CONTOURS) {
                    ignoredSmall++
                } else {
                    blobs += measure(contour, area)
                }
                contour.release()
            }
        } finally {
            region.release()
        }

        val selection = BallCandidateFilter.evaluate(blobs, width.toDouble() * height, s.filters)
        val result = BallFrameResult(
            frameNumber = ++frameNumber,
            captureTimeNanos = captureTimeNanos,
            processingStartNanos = start,
            publishedNanos = System.nanoTime(),
            widthPx = width,
            heightPx = height,
            roi = roi,
            settingsVersion = versioned.version,
            contourCount = contourCount,
            ignoredSmallCount = ignoredSmall,
            selection = selection,
        )

        if (!s.previewEnabled) {
            latestRef.set(result)
            return null
        }
        if (s.previewMode == BallPreviewMode.MASK) {
            // The displayed image is the frame Mat after processing: black it
            // out and paint the thresholded pixels white inside the ROI.
            frame.setTo(BLACK)
            val displayRegion = frame.submat(Rect(roi.left, roi.top, roi.width, roi.height))
            try {
                displayRegion.setTo(WHITE, mask)
            } finally {
                displayRegion.release()
            }
        }
        latestRef.set(result)
        return DrawContext(result, s.previewMode)
    }

    private fun measure(contour: MatOfPoint, area: Double): BlobMeasurement {
        val points = contour.toArray()
        val contour2f = MatOfPoint2f(*points)
        val hullIndices = MatOfInt()
        try {
            val moments = Imgproc.moments(contour)
            val box = Imgproc.boundingRect(contour)
            val centroidX = if (moments.m00 != 0.0) moments.m10 / moments.m00 else box.x + box.width / 2.0
            val centroidY = if (moments.m00 != 0.0) moments.m01 / moments.m00 else box.y + box.height / 2.0
            val center = Point()
            val radius = FloatArray(1)
            Imgproc.minEnclosingCircle(contour2f, center, radius)
            val perimeter = Imgproc.arcLength(contour2f, true)
            val rotated = Imgproc.minAreaRect(contour2f)
            val longSide = maxOf(1.0, maxOf(rotated.size.width, rotated.size.height))
            val shortSide = maxOf(1.0, minOf(rotated.size.width, rotated.size.height))
            Imgproc.convexHull(contour, hullIndices)
            val hullPoints = hullIndices.toArray().map { points[it] }.toTypedArray()
            val hull = MatOfPoint(*hullPoints)
            val hullArea = try {
                maxOf(1.0, Imgproc.contourArea(hull))
            } finally {
                hull.release()
            }
            val safeArea = maxOf(1.0, area)
            return BlobMeasurement(
                centroidXPx = centroidX,
                centroidYPx = centroidY,
                enclosingRadiusPx = radius[0].toDouble(),
                contourAreaPx = area,
                boxLeftPx = box.x,
                boxTopPx = box.y,
                boxWidthPx = box.width,
                boxHeightPx = box.height,
                circularity = if (perimeter > 0.0) 4 * Math.PI * safeArea / (perimeter * perimeter) else 0.0,
                aspectRatio = longSide / shortSide,
                density = safeArea / hullArea,
            )
        } finally {
            contour2f.release()
            hullIndices.release()
        }
    }

    private fun erode(size: Int) {
        if (size <= 0) return
        if (erodeKernel == null || erodeKernelSize != size) {
            erodeKernel?.release()
            erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(size.toDouble(), size.toDouble()))
            erodeKernelSize = size
        }
        Imgproc.erode(mask, mask, erodeKernel)
    }

    private fun dilate(size: Int) {
        if (size <= 0) return
        if (dilateKernel == null || dilateKernelSize != size) {
            dilateKernel?.release()
            dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(size.toDouble(), size.toDouble()))
            dilateKernelSize = size
        }
        Imgproc.dilate(mask, mask, dilateKernel)
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val ctx = userContext as? DrawContext ?: return
        try {
            draw(canvas, scaleBmpPxToCanvasPx, scaleCanvasDensity, ctx)
        } catch (t: Throwable) {
            lastError = "draw ${t.javaClass.simpleName}: ${t.message}"
        }
    }

    private fun draw(canvas: Canvas, scale: Float, density: Float, ctx: DrawContext) {
        val result = ctx.result
        val selection = result.selection
        textPaint.textSize = 14f * density
        val label = "${ctx.mode} ${result.widthPx}x${result.heightPx} #${result.frameNumber} " +
            "${selection.acceptedCount}/${selection.evaluatedCount} ok"
        canvas.drawText(label, 8f * density, 20f * density, textPaint)
        if (ctx.mode == BallPreviewMode.ORIGINAL) return

        roiPaint.strokeWidth = 2f * density
        val roi = result.roi
        canvas.drawRect(
            roi.left * scale, roi.top * scale,
            (roi.left + roi.width) * scale, (roi.top + roi.height) * scale,
            roiPaint,
        )
        acceptedPaint.strokeWidth = 2f * density
        rejectedPaint.strokeWidth = 1.5f * density
        for (candidate in selection.candidates) {
            if (candidate === selection.selected) continue
            val blob = candidate.blob
            val x = blob.centroidXPx.toFloat() * scale
            val y = blob.centroidYPx.toFloat() * scale
            val r = blob.enclosingRadiusPx.toFloat() * scale
            if (candidate.accepted) {
                canvas.drawCircle(x, y, r, acceptedPaint)
            } else {
                canvas.drawCircle(x, y, r, rejectedPaint)
                canvas.drawText(candidate.rejection?.label ?: "", x + r, y, textPaint)
            }
        }
        selection.selected?.blob?.let { blob ->
            selectedPaint.strokeWidth = 5f * density
            val x = blob.centroidXPx.toFloat() * scale
            val y = blob.centroidYPx.toFloat() * scale
            val r = blob.enclosingRadiusPx.toFloat() * scale
            canvas.drawCircle(x, y, r, selectedPaint)
            canvas.drawLine(x - r, y, x + r, y, selectedPaint)
            canvas.drawLine(x, y - r, x, y + r, selectedPaint)
            canvas.drawText("TARGET", x - r, y - r - 4f * density, textPaint)
        }
    }

    /** Frees native buffers. Call only after the camera has stopped delivering frames. */
    fun releaseBuffers() {
        converted.release()
        mask.release()
        hierarchy.release()
        erodeKernel?.release()
        dilateKernel?.release()
    }

    private companion object {
        const val MAX_MEASURED_CONTOURS = 64
        val BLACK = Scalar(0.0, 0.0, 0.0, 255.0)
        val WHITE = Scalar(255.0, 255.0, 255.0, 255.0)

        fun strokePaint(color: Int) = Paint().apply {
            this.color = color
            style = Paint.Style.STROKE
            isAntiAlias = true
        }

        fun lensReportOf(width: Int, height: Int, calibration: CameraCalibration?): LensReport {
            if (calibration == null || calibration.isFake || calibration.isDegenerate) {
                return LensReport(CalibrationSource.UNAVAILABLE, null)
            }
            val intrinsics = LensIntrinsics(
                widthPx = width,
                heightPx = height,
                fx = calibration.focalLengthX.toDouble(),
                fy = calibration.focalLengthY.toDouble(),
                cx = calibration.principalPointX.toDouble(),
                cy = calibration.principalPointY.toDouble(),
                distortion = calibration.distortionCoefficients.map { it.toDouble() },
            )
            val scaledFrom = calibration.resolutionScaledFrom
            return if (scaledFrom != null) {
                LensReport(CalibrationSource.SCALED, intrinsics, "${scaledFrom.width}x${scaledFrom.height}")
            } else {
                LensReport(CalibrationSource.EXACT, intrinsics)
            }
        }
    }
}
