package org.firstinspires.ftc.teamcode.vision

import com.bylazar.configurables.annotations.Configurable

/**
 * Panels-tunable settings for the USB ball camera, persisted by ConfigStore
 * under `ballVision`.
 *
 * Panels writes these statics from its websocket thread. Nothing reads them
 * there or on the camera thread: the robot loop copies them once per tick into
 * an immutable [BallVisionSettings], and only that snapshot is handed to the
 * camera and camera-control threads. Fields marked **restart** are captured
 * when the OpMode initializes; changing them mid-run is reported, not applied.
 *
 * Saved values win over the compiled defaults below: every OpMode init resets
 * these fields to the defaults, then applies `ballVision.*` keys from
 * `/sdcard/FIRST/config/tuning.properties`. Set [resetToDefaults] (or delete
 * the keys and restart) to return to the defaults. The starting values are
 * the SDK's generic YCrCb yellow range and permissive shape filters — not yet
 * tuned against pollen.
 */
@Configurable
object BallVisionConfig {

    const val COLOR_SPACE_YCRCB = 0
    const val COLOR_SPACE_HSV = 1

    const val PREVIEW_OVERLAY = 0
    const val PREVIEW_ORIGINAL = 1
    const val PREVIEW_MASK = 2

    const val STREAM_MJPEG = 0
    const val STREAM_YUY2 = 1

    private const val DEFAULT_COLOR_SPACE = COLOR_SPACE_YCRCB
    private const val DEFAULT_CHANNEL0_MIN = 32
    private const val DEFAULT_CHANNEL0_MAX = 255
    private const val DEFAULT_CHANNEL1_MIN = 128
    private const val DEFAULT_CHANNEL1_MAX = 170
    private const val DEFAULT_CHANNEL2_MIN = 0
    private const val DEFAULT_CHANNEL2_MAX = 120
    private const val DEFAULT_ROI_LEFT = 0.0
    private const val DEFAULT_ROI_TOP = 0.0
    private const val DEFAULT_ROI_RIGHT = 1.0
    private const val DEFAULT_ROI_BOTTOM = 1.0
    private const val DEFAULT_BLUR_KERNEL_PX = 5
    private const val DEFAULT_ERODE_KERNEL_PX = 3
    private const val DEFAULT_DILATE_KERNEL_PX = 3
    private const val DEFAULT_MORPH_CLOSING = false
    private const val DEFAULT_EXTERNAL_CONTOURS_ONLY = true
    private const val DEFAULT_MIN_AREA_PERCENT = 0.05
    private const val DEFAULT_MAX_AREA_PERCENT = 40.0
    private const val DEFAULT_MIN_CIRCULARITY = 0.5
    private const val DEFAULT_MAX_ASPECT_RATIO = 2.5
    private const val DEFAULT_MIN_DENSITY = 0.7
    private const val DEFAULT_MAX_PUBLISHED_CANDIDATES = 8
    private const val DEFAULT_MAX_OBSERVATION_AGE_MS = 150.0
    private const val DEFAULT_PREVIEW_ENABLED = true
    private const val DEFAULT_PREVIEW_MODE = PREVIEW_OVERLAY
    private const val DEFAULT_EXPOSURE_MANUAL = false
    private const val DEFAULT_EXPOSURE_MICROS = 5000
    private const val DEFAULT_GAIN = -1
    private const val DEFAULT_WHITE_BALANCE_MANUAL = false
    private const val DEFAULT_WHITE_BALANCE_KELVIN = 4600
    private const val DEFAULT_RESOLUTION_WIDTH = 640
    private const val DEFAULT_RESOLUTION_HEIGHT = 480
    private const val DEFAULT_STREAM_FORMAT = STREAM_MJPEG
    private const val DEFAULT_RESET_TO_DEFAULTS = false

    /** 0 = YCrCb (channels Y, Cr, Cb), 1 = HSV (OpenCV H 0–180, S, V). */
    @JvmField var colorSpace: Int = DEFAULT_COLOR_SPACE

    /** Inclusive threshold per channel of [colorSpace], 0–255 (HSV hue 0–180). */
    @JvmField var channel0Min: Int = DEFAULT_CHANNEL0_MIN
    @JvmField var channel0Max: Int = DEFAULT_CHANNEL0_MAX
    @JvmField var channel1Min: Int = DEFAULT_CHANNEL1_MIN
    @JvmField var channel1Max: Int = DEFAULT_CHANNEL1_MAX
    @JvmField var channel2Min: Int = DEFAULT_CHANNEL2_MIN
    @JvmField var channel2Max: Int = DEFAULT_CHANNEL2_MAX

    /** Search region as fractions of the frame, origin top-left: 0 ≤ left < right ≤ 1, 0 ≤ top < bottom ≤ 1. */
    @JvmField var roiLeft: Double = DEFAULT_ROI_LEFT
    @JvmField var roiTop: Double = DEFAULT_ROI_TOP
    @JvmField var roiRight: Double = DEFAULT_ROI_RIGHT
    @JvmField var roiBottom: Double = DEFAULT_ROI_BOTTOM

    /** Kernel sizes in processed-frame pixels; 0 disables. Halve them at 320×240 for equivalent filtering. */
    @JvmField var blurKernelPx: Int = DEFAULT_BLUR_KERNEL_PX
    @JvmField var erodeKernelPx: Int = DEFAULT_ERODE_KERNEL_PX
    @JvmField var dilateKernelPx: Int = DEFAULT_DILATE_KERNEL_PX

    /** false = erode then dilate (removes speckles); true = dilate then erode (fills holes). */
    @JvmField var morphClosing: Boolean = DEFAULT_MORPH_CLOSING

    /** Ignore contours nested inside another contour (glare holes inside a ball). */
    @JvmField var externalContoursOnly: Boolean = DEFAULT_EXTERNAL_CONTOURS_ONLY

    /** Contour-area limits as a percentage of the full frame, so they survive a resolution change. */
    @JvmField var minAreaPercent: Double = DEFAULT_MIN_AREA_PERCENT
    @JvmField var maxAreaPercent: Double = DEFAULT_MAX_AREA_PERCENT

    /** 4π·area/perimeter²; 1.0 is a perfect circle. */
    @JvmField var minCircularity: Double = DEFAULT_MIN_CIRCULARITY

    /** Long/short side of the minimum-area rectangle; 1.0 is square. */
    @JvmField var maxAspectRatio: Double = DEFAULT_MAX_ASPECT_RATIO

    /** Contour area / convex-hull area; low for merged or notched blobs. */
    @JvmField var minDensity: Double = DEFAULT_MIN_DENSITY

    /** Candidates published per frame (accepted first). Selection always sees every candidate. */
    @JvmField var maxPublishedCandidates: Int = DEFAULT_MAX_PUBLISHED_CANDIDATES

    /** Observations whose camera capture time is older than this are cleared. */
    @JvmField var maxObservationAgeMs: Double = DEFAULT_MAX_OBSERVATION_AGE_MS

    /** Robot Controller live view. Off pauses rendering and skips mask/overlay work. */
    @JvmField var previewEnabled: Boolean = DEFAULT_PREVIEW_ENABLED

    /** 0 = detection overlay, 1 = original image, 2 = threshold mask. */
    @JvmField var previewMode: Int = DEFAULT_PREVIEW_MODE

    /** Manual exposure; false leaves the camera's automatic exposure in charge. */
    @JvmField var exposureManual: Boolean = DEFAULT_EXPOSURE_MANUAL

    /** Manual exposure time in microseconds, clamped to the probed range. */
    @JvmField var exposureMicros: Int = DEFAULT_EXPOSURE_MICROS

    /** Sensor gain, applied only if the probe finds a gain range and exposure is manual; -1 leaves it alone. */
    @JvmField var gain: Int = DEFAULT_GAIN

    @JvmField var whiteBalanceManual: Boolean = DEFAULT_WHITE_BALANCE_MANUAL

    /** Manual white-balance temperature, clamped to the probed range. */
    @JvmField var whiteBalanceKelvin: Int = DEFAULT_WHITE_BALANCE_KELVIN

    /** **restart** Capture resolution. goBILDA lists 640×480 and 320×240 MJPEG at up to 120 fps. */
    @JvmField var resolutionWidth: Int = DEFAULT_RESOLUTION_WIDTH
    @JvmField var resolutionHeight: Int = DEFAULT_RESOLUTION_HEIGHT

    /** **restart** 0 = MJPEG, 1 = YUY2 (goBILDA lists YUY2 only at 1280-wide, 10 fps). */
    @JvmField var streamFormat: Int = DEFAULT_STREAM_FORMAT

    /** Set true to restore every field above to its compiled default on the next robot loop. */
    @JvmField var resetToDefaults: Boolean = DEFAULT_RESET_TO_DEFAULTS

    fun resetDefaults() {
        colorSpace = DEFAULT_COLOR_SPACE
        channel0Min = DEFAULT_CHANNEL0_MIN
        channel0Max = DEFAULT_CHANNEL0_MAX
        channel1Min = DEFAULT_CHANNEL1_MIN
        channel1Max = DEFAULT_CHANNEL1_MAX
        channel2Min = DEFAULT_CHANNEL2_MIN
        channel2Max = DEFAULT_CHANNEL2_MAX
        roiLeft = DEFAULT_ROI_LEFT
        roiTop = DEFAULT_ROI_TOP
        roiRight = DEFAULT_ROI_RIGHT
        roiBottom = DEFAULT_ROI_BOTTOM
        blurKernelPx = DEFAULT_BLUR_KERNEL_PX
        erodeKernelPx = DEFAULT_ERODE_KERNEL_PX
        dilateKernelPx = DEFAULT_DILATE_KERNEL_PX
        morphClosing = DEFAULT_MORPH_CLOSING
        externalContoursOnly = DEFAULT_EXTERNAL_CONTOURS_ONLY
        minAreaPercent = DEFAULT_MIN_AREA_PERCENT
        maxAreaPercent = DEFAULT_MAX_AREA_PERCENT
        minCircularity = DEFAULT_MIN_CIRCULARITY
        maxAspectRatio = DEFAULT_MAX_ASPECT_RATIO
        minDensity = DEFAULT_MIN_DENSITY
        maxPublishedCandidates = DEFAULT_MAX_PUBLISHED_CANDIDATES
        maxObservationAgeMs = DEFAULT_MAX_OBSERVATION_AGE_MS
        previewEnabled = DEFAULT_PREVIEW_ENABLED
        previewMode = DEFAULT_PREVIEW_MODE
        exposureManual = DEFAULT_EXPOSURE_MANUAL
        exposureMicros = DEFAULT_EXPOSURE_MICROS
        gain = DEFAULT_GAIN
        whiteBalanceManual = DEFAULT_WHITE_BALANCE_MANUAL
        whiteBalanceKelvin = DEFAULT_WHITE_BALANCE_KELVIN
        resolutionWidth = DEFAULT_RESOLUTION_WIDTH
        resolutionHeight = DEFAULT_RESOLUTION_HEIGHT
        streamFormat = DEFAULT_STREAM_FORMAT
        resetToDefaults = DEFAULT_RESET_TO_DEFAULTS
    }

    /** Compiled defaults keyed by field name, for lab records that flag tuned values. */
    fun compiledDefaults(): Map<String, Any> = linkedMapOf(
        "colorSpace" to DEFAULT_COLOR_SPACE,
        "channel0Min" to DEFAULT_CHANNEL0_MIN,
        "channel0Max" to DEFAULT_CHANNEL0_MAX,
        "channel1Min" to DEFAULT_CHANNEL1_MIN,
        "channel1Max" to DEFAULT_CHANNEL1_MAX,
        "channel2Min" to DEFAULT_CHANNEL2_MIN,
        "channel2Max" to DEFAULT_CHANNEL2_MAX,
        "roiLeft" to DEFAULT_ROI_LEFT,
        "roiTop" to DEFAULT_ROI_TOP,
        "roiRight" to DEFAULT_ROI_RIGHT,
        "roiBottom" to DEFAULT_ROI_BOTTOM,
        "blurKernelPx" to DEFAULT_BLUR_KERNEL_PX,
        "erodeKernelPx" to DEFAULT_ERODE_KERNEL_PX,
        "dilateKernelPx" to DEFAULT_DILATE_KERNEL_PX,
        "morphClosing" to DEFAULT_MORPH_CLOSING,
        "externalContoursOnly" to DEFAULT_EXTERNAL_CONTOURS_ONLY,
        "minAreaPercent" to DEFAULT_MIN_AREA_PERCENT,
        "maxAreaPercent" to DEFAULT_MAX_AREA_PERCENT,
        "minCircularity" to DEFAULT_MIN_CIRCULARITY,
        "maxAspectRatio" to DEFAULT_MAX_ASPECT_RATIO,
        "minDensity" to DEFAULT_MIN_DENSITY,
        "maxPublishedCandidates" to DEFAULT_MAX_PUBLISHED_CANDIDATES,
        "maxObservationAgeMs" to DEFAULT_MAX_OBSERVATION_AGE_MS,
        "previewEnabled" to DEFAULT_PREVIEW_ENABLED,
        "previewMode" to DEFAULT_PREVIEW_MODE,
        "exposureManual" to DEFAULT_EXPOSURE_MANUAL,
        "exposureMicros" to DEFAULT_EXPOSURE_MICROS,
        "gain" to DEFAULT_GAIN,
        "whiteBalanceManual" to DEFAULT_WHITE_BALANCE_MANUAL,
        "whiteBalanceKelvin" to DEFAULT_WHITE_BALANCE_KELVIN,
        "resolutionWidth" to DEFAULT_RESOLUTION_WIDTH,
        "resolutionHeight" to DEFAULT_RESOLUTION_HEIGHT,
        "streamFormat" to DEFAULT_STREAM_FORMAT,
        "resetToDefaults" to DEFAULT_RESET_TO_DEFAULTS,
    )
}
