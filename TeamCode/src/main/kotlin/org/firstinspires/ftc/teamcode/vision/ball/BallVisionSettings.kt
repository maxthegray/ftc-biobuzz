package org.firstinspires.ftc.teamcode.vision.ball

import kotlin.math.roundToInt

enum class BallColorSpace { YCRCB, HSV }

enum class BallPreviewMode { OVERLAY, ORIGINAL, MASK }

enum class BallStreamFormat { MJPEG, YUY2 }

/** Normalized search region, origin top-left, already validated. */
data class NormalizedRoi(val left: Double, val top: Double, val right: Double, val bottom: Double) {

    /** Pixel rectangle inside a [width]×[height] frame; always at least 1×1. */
    fun toPixels(width: Int, height: Int): PixelRect {
        val l = (left * width).roundToInt().coerceIn(0, width - 1)
        val t = (top * height).roundToInt().coerceIn(0, height - 1)
        val r = (right * width).roundToInt().coerceIn(l + 1, width)
        val b = (bottom * height).roundToInt().coerceIn(t + 1, height)
        return PixelRect(l, t, r - l, b - t)
    }

    companion object {
        val FULL_FRAME = NormalizedRoi(0.0, 0.0, 1.0, 1.0)
    }
}

data class PixelRect(val left: Int, val top: Int, val width: Int, val height: Int)

data class BlobFilterSettings(
    val minAreaPercent: Double,
    val maxAreaPercent: Double,
    val minCircularity: Double,
    val maxAspectRatio: Double,
    val minDensity: Double,
    val maxPublishedCandidates: Int,
)

/** Everything the camera-thread processor needs; immutable so it can cross threads. */
data class DetectionSettings(
    val colorSpace: BallColorSpace,
    val lower: ChannelTriple,
    val upper: ChannelTriple,
    val roi: NormalizedRoi,
    val blurKernelPx: Int,
    val erodeKernelPx: Int,
    val dilateKernelPx: Int,
    val morphClosing: Boolean,
    val externalContoursOnly: Boolean,
    val filters: BlobFilterSettings,
    val previewEnabled: Boolean,
    val previewMode: BallPreviewMode,
)

data class ChannelTriple(val c0: Int, val c1: Int, val c2: Int)

/** Device image controls, applied on the camera-control thread. */
data class CameraImageRequest(
    val exposureManual: Boolean,
    val exposureMicros: Long,
    /** Negative leaves gain untouched. */
    val gain: Int,
    val whiteBalanceManual: Boolean,
    val whiteBalanceKelvin: Int,
    val liveViewEnabled: Boolean,
)

/** Settings that only take effect when the camera is opened. */
data class StreamSettings(val width: Int, val height: Int, val format: BallStreamFormat) {
    override fun toString(): String = "${width}x$height $format"

    companion object {
        /**
         * Modes goBILDA lists for the 3122-0004-0001 camera (user guide, May 14
         * 2026). Asking EasyOpenCV for anything else fails inside the camera-open
         * thread, which the library turns into a robot E-stop, so the subsystem
         * refuses other modes at init instead.
         */
        val GOBILDA_3122_0004_0001_MODES: List<StreamSettings> = listOf(
            StreamSettings(1280, 800, BallStreamFormat.MJPEG),
            StreamSettings(1280, 720, BallStreamFormat.MJPEG),
            StreamSettings(800, 600, BallStreamFormat.MJPEG),
            StreamSettings(640, 480, BallStreamFormat.MJPEG),
            StreamSettings(320, 240, BallStreamFormat.MJPEG),
            StreamSettings(1280, 800, BallStreamFormat.YUY2),
            StreamSettings(1280, 720, BallStreamFormat.YUY2),
        )
    }
}

/**
 * One validated copy of [BallVisionConfig], taken on the robot thread. Invalid
 * Panels input is clamped or replaced here, so downstream code never sees an
 * inverted range, an empty ROI, or an even blur kernel.
 */
data class BallVisionSettings(
    val detection: DetectionSettings,
    val camera: CameraImageRequest,
    val stream: StreamSettings,
    val maxObservationAgeMs: Double,
) {
    companion object {
        private const val MAX_KERNEL_PX = 31
        private const val FALLBACK_MAX_AGE_MS = 150.0

        fun fromConfig(): BallVisionSettings = with(BallVisionConfig) {
            val space = if (colorSpace == BallVisionConfig.COLOR_SPACE_HSV) BallColorSpace.HSV else BallColorSpace.YCRCB
            val c0Limit = if (space == BallColorSpace.HSV) 180 else 255
            val (c0Lo, c0Hi) = orderedChannel(channel0Min, channel0Max, c0Limit)
            val (c1Lo, c1Hi) = orderedChannel(channel1Min, channel1Max, 255)
            val (c2Lo, c2Hi) = orderedChannel(channel2Min, channel2Max, 255)
            val minArea = finiteOr(minAreaPercent, 0.0).coerceIn(0.0, 100.0)
            val maxArea = finiteOr(maxAreaPercent, 100.0).coerceIn(minArea, 100.0)
            BallVisionSettings(
                detection = DetectionSettings(
                    colorSpace = space,
                    lower = ChannelTriple(c0Lo, c1Lo, c2Lo),
                    upper = ChannelTriple(c0Hi, c1Hi, c2Hi),
                    roi = roi(roiLeft, roiTop, roiRight, roiBottom),
                    blurKernelPx = oddKernel(blurKernelPx),
                    erodeKernelPx = morphKernel(erodeKernelPx),
                    dilateKernelPx = morphKernel(dilateKernelPx),
                    morphClosing = morphClosing,
                    externalContoursOnly = externalContoursOnly,
                    filters = BlobFilterSettings(
                        minAreaPercent = minArea,
                        maxAreaPercent = maxArea,
                        minCircularity = finiteOr(minCircularity, 0.0).coerceIn(0.0, 1.0),
                        maxAspectRatio = finiteOr(maxAspectRatio, Double.MAX_VALUE).coerceAtLeast(1.0),
                        minDensity = finiteOr(minDensity, 0.0).coerceIn(0.0, 1.0),
                        maxPublishedCandidates = maxPublishedCandidates.coerceIn(0, 32),
                    ),
                    previewEnabled = previewEnabled,
                    previewMode = previewModeOf(previewMode),
                ),
                camera = CameraImageRequest(
                    exposureManual = exposureManual,
                    exposureMicros = exposureMicros.coerceAtLeast(1).toLong(),
                    gain = if (gain < 0) -1 else gain,
                    whiteBalanceManual = whiteBalanceManual,
                    whiteBalanceKelvin = whiteBalanceKelvin.coerceAtLeast(1),
                    liveViewEnabled = previewEnabled,
                ),
                stream = StreamSettings(
                    width = resolutionWidth.coerceAtLeast(1),
                    height = resolutionHeight.coerceAtLeast(1),
                    format = if (streamFormat == BallVisionConfig.STREAM_YUY2) BallStreamFormat.YUY2 else BallStreamFormat.MJPEG,
                ),
                maxObservationAgeMs = finiteOr(maxObservationAgeMs, FALLBACK_MAX_AGE_MS)
                    .takeIf { it > 0.0 } ?: FALLBACK_MAX_AGE_MS,
            )
        }

        fun previewModeOf(code: Int): BallPreviewMode = when (code) {
            BallVisionConfig.PREVIEW_ORIGINAL -> BallPreviewMode.ORIGINAL
            BallVisionConfig.PREVIEW_MASK -> BallPreviewMode.MASK
            else -> BallPreviewMode.OVERLAY
        }

        private fun orderedChannel(min: Int, max: Int, limit: Int): Pair<Int, Int> {
            val a = min.coerceIn(0, limit)
            val b = max.coerceIn(0, limit)
            return if (a <= b) a to b else b to a
        }

        /** An unusable ROI falls back to the full frame rather than a sliver. */
        private fun roi(left: Double, top: Double, right: Double, bottom: Double): NormalizedRoi {
            if (!(left.isFinite() && top.isFinite() && right.isFinite() && bottom.isFinite())) {
                return NormalizedRoi.FULL_FRAME
            }
            val l = left.coerceIn(0.0, 1.0)
            val t = top.coerceIn(0.0, 1.0)
            val r = right.coerceIn(0.0, 1.0)
            val b = bottom.coerceIn(0.0, 1.0)
            return if (r > l && b > t) NormalizedRoi(l, t, r, b) else NormalizedRoi.FULL_FRAME
        }

        /** GaussianBlur needs an odd kernel; 0 disables. */
        private fun oddKernel(px: Int): Int {
            if (px <= 0) return 0
            val clamped = px.coerceAtMost(MAX_KERNEL_PX)
            return if (clamped % 2 == 0) clamped + 1 else clamped
        }

        private fun morphKernel(px: Int): Int = px.coerceIn(0, MAX_KERNEL_PX)

        private fun finiteOr(value: Double, fallback: Double): Double = if (value.isFinite()) value else fallback
    }
}
