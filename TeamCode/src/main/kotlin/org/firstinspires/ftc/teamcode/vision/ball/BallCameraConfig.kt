package org.firstinspires.ftc.teamcode.vision.ball

import com.bylazar.configurables.annotations.Configurable

/**
 * Panels-tunable settings for the USB ball camera, persisted by ConfigStore
 * under `ballVision`. Tentative values from the September 18, 2026 tuning
 * session.
 *
 * Fields marked **init** are built into the SDK's ColorBlobLocatorProcessor
 * when the OpMode initializes: edit them in Panels, then stop and re-init.
 * The rest apply on the next robot loop.
 *
 * Saved values win over the compiled defaults below: every OpMode init resets
 * these fields to the defaults, then applies `ballVision.*` keys from
 * `/sdcard/FIRST/config/tuning.properties`. Set [resetToDefaults] (or delete
 * the keys and restart) to return to the defaults.
 *
 * The `mount*` fields record where the lens sits on the robot. Nothing on the
 * robot uses them: [BallCameraSubsystem] logs them every tick under
 * `BallCamera/mount/…` so MaxScope can place detections on the field.
 */
@Configurable
object BallCameraConfig {
    const val COLOR_SPACE_YCRCB = 0
    const val COLOR_SPACE_HSV = 1

    private const val DEFAULT_COLOR_SPACE = COLOR_SPACE_YCRCB
    private const val DEFAULT_CHANNEL0_MIN = 125
    private const val DEFAULT_CHANNEL0_MAX = 255
    private const val DEFAULT_CHANNEL1_MIN = 130
    private const val DEFAULT_CHANNEL1_MAX = 170
    private const val DEFAULT_CHANNEL2_MIN = 50
    private const val DEFAULT_CHANNEL2_MAX = 110
    private const val DEFAULT_ROI_LEFT = 0.0
    private const val DEFAULT_ROI_TOP = 0.0
    private const val DEFAULT_ROI_RIGHT = 1.0
    private const val DEFAULT_ROI_BOTTOM = 1.0
    private const val DEFAULT_BLUR_KERNEL_PX = 5
    private const val DEFAULT_ERODE_KERNEL_PX = 3
    private const val DEFAULT_DILATE_KERNEL_PX = 3
    private const val DEFAULT_MORPH_CLOSING = false
    private const val DEFAULT_EXTERNAL_CONTOURS_ONLY = true
    private const val DEFAULT_EXPOSURE_MANUAL = true
    private const val DEFAULT_EXPOSURE_MICROS = 6000
    private const val DEFAULT_MIN_AREA_PERCENT = 0.05
    private const val DEFAULT_MAX_AREA_PERCENT = 40.0
    private const val DEFAULT_MIN_CIRCULARITY = 1.0
    private const val DEFAULT_MAX_ASPECT_RATIO = 2.5
    private const val DEFAULT_MIN_DENSITY = 0.7
    private const val DEFAULT_MAX_OBSERVATION_AGE_MS = 150.0
    private const val DEFAULT_PREVIEW_ENABLED = true
    private const val DEFAULT_MOUNT_HEIGHT_IN = 0.0
    private const val DEFAULT_MOUNT_PITCH_DOWN_DEG = 0.0
    private const val DEFAULT_MOUNT_FORWARD_IN = 0.0
    private const val DEFAULT_MOUNT_LEFT_IN = 0.0
    private const val DEFAULT_MOUNT_YAW_DEG = 0.0
    private const val DEFAULT_MOUNT_MEASURED = false
    private const val DEFAULT_RESET_TO_DEFAULTS = false

    /** **init** 0 = YCrCb (channels Y, Cr, Cb), 1 = HSV (OpenCV H 0–180, S, V). */
    @JvmField var colorSpace: Int = DEFAULT_COLOR_SPACE

    /** **init** Inclusive threshold per channel of [colorSpace], 0–255 (HSV hue 0–180). */
    @JvmField var channel0Min: Int = DEFAULT_CHANNEL0_MIN
    @JvmField var channel0Max: Int = DEFAULT_CHANNEL0_MAX
    @JvmField var channel1Min: Int = DEFAULT_CHANNEL1_MIN
    @JvmField var channel1Max: Int = DEFAULT_CHANNEL1_MAX
    @JvmField var channel2Min: Int = DEFAULT_CHANNEL2_MIN
    @JvmField var channel2Max: Int = DEFAULT_CHANNEL2_MAX

    /** **init** Search region as fractions of the frame, origin top-left: 0 ≤ left < right ≤ 1, 0 ≤ top < bottom ≤ 1. */
    @JvmField var roiLeft: Double = DEFAULT_ROI_LEFT
    @JvmField var roiTop: Double = DEFAULT_ROI_TOP
    @JvmField var roiRight: Double = DEFAULT_ROI_RIGHT
    @JvmField var roiBottom: Double = DEFAULT_ROI_BOTTOM

    /** **init** Kernel sizes in pixels; 0 disables. */
    @JvmField var blurKernelPx: Int = DEFAULT_BLUR_KERNEL_PX
    @JvmField var erodeKernelPx: Int = DEFAULT_ERODE_KERNEL_PX
    @JvmField var dilateKernelPx: Int = DEFAULT_DILATE_KERNEL_PX

    /** **init** false = erode then dilate (removes speckles); true = dilate then erode (fills holes). */
    @JvmField var morphClosing: Boolean = DEFAULT_MORPH_CLOSING

    /** **init** Ignore contours nested inside another contour (glare holes inside a ball). */
    @JvmField var externalContoursOnly: Boolean = DEFAULT_EXTERNAL_CONTOURS_ONLY

    /** **init** Manual exposure keeps the color threshold valid as the lighting changes; false leaves auto exposure on. */
    @JvmField var exposureManual: Boolean = DEFAULT_EXPOSURE_MANUAL
    @JvmField var exposureMicros: Int = DEFAULT_EXPOSURE_MICROS

    /** Contour-area limits as a percentage of the full frame. */
    @JvmField var minAreaPercent: Double = DEFAULT_MIN_AREA_PERCENT
    @JvmField var maxAreaPercent: Double = DEFAULT_MAX_AREA_PERCENT

    /** 4π·area/perimeter²; 1.0 is a perfect circle, which a traced contour rarely reaches. */
    @JvmField var minCircularity: Double = DEFAULT_MIN_CIRCULARITY

    /** Long/short side of the minimum-area rectangle; 1.0 is square. */
    @JvmField var maxAspectRatio: Double = DEFAULT_MAX_ASPECT_RATIO

    /** Contour area / convex-hull area; low for merged or notched blobs. */
    @JvmField var minDensity: Double = DEFAULT_MIN_DENSITY

    /** Detections clear when no new frame has arrived for this long. */
    @JvmField var maxObservationAgeMs: Double = DEFAULT_MAX_OBSERVATION_AGE_MS

    /** Robot Controller live view. Off saves CPU. */
    @JvmField var previewEnabled: Boolean = DEFAULT_PREVIEW_ENABLED

    /** Lens centre above the floor, inches. */
    @JvmField var mountHeightIn: Double = DEFAULT_MOUNT_HEIGHT_IN

    /** Optical axis below horizontal, degrees; positive tilts the camera down. */
    @JvmField var mountPitchDownDeg: Double = DEFAULT_MOUNT_PITCH_DOWN_DEG

    /** Lens position from the robot's pose point (its centre): + toward the robot's front, inches. */
    @JvmField var mountForwardIn: Double = DEFAULT_MOUNT_FORWARD_IN

    /** Lens position from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var mountLeftIn: Double = DEFAULT_MOUNT_LEFT_IN

    /** Optical axis from the robot's front, degrees, counterclockwise-positive like Pedro headings. */
    @JvmField var mountYawDeg: Double = DEFAULT_MOUNT_YAW_DEG

    /** True once the mount values above describe the camera as mounted. */
    @JvmField var mountMeasured: Boolean = DEFAULT_MOUNT_MEASURED

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
        exposureManual = DEFAULT_EXPOSURE_MANUAL
        exposureMicros = DEFAULT_EXPOSURE_MICROS
        minAreaPercent = DEFAULT_MIN_AREA_PERCENT
        maxAreaPercent = DEFAULT_MAX_AREA_PERCENT
        minCircularity = DEFAULT_MIN_CIRCULARITY
        maxAspectRatio = DEFAULT_MAX_ASPECT_RATIO
        minDensity = DEFAULT_MIN_DENSITY
        maxObservationAgeMs = DEFAULT_MAX_OBSERVATION_AGE_MS
        previewEnabled = DEFAULT_PREVIEW_ENABLED
        mountHeightIn = DEFAULT_MOUNT_HEIGHT_IN
        mountPitchDownDeg = DEFAULT_MOUNT_PITCH_DOWN_DEG
        mountForwardIn = DEFAULT_MOUNT_FORWARD_IN
        mountLeftIn = DEFAULT_MOUNT_LEFT_IN
        mountYawDeg = DEFAULT_MOUNT_YAW_DEG
        mountMeasured = DEFAULT_MOUNT_MEASURED
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
        "exposureManual" to DEFAULT_EXPOSURE_MANUAL,
        "exposureMicros" to DEFAULT_EXPOSURE_MICROS,
        "minAreaPercent" to DEFAULT_MIN_AREA_PERCENT,
        "maxAreaPercent" to DEFAULT_MAX_AREA_PERCENT,
        "minCircularity" to DEFAULT_MIN_CIRCULARITY,
        "maxAspectRatio" to DEFAULT_MAX_ASPECT_RATIO,
        "minDensity" to DEFAULT_MIN_DENSITY,
        "maxObservationAgeMs" to DEFAULT_MAX_OBSERVATION_AGE_MS,
        "previewEnabled" to DEFAULT_PREVIEW_ENABLED,
        "mountHeightIn" to DEFAULT_MOUNT_HEIGHT_IN,
        "mountPitchDownDeg" to DEFAULT_MOUNT_PITCH_DOWN_DEG,
        "mountForwardIn" to DEFAULT_MOUNT_FORWARD_IN,
        "mountLeftIn" to DEFAULT_MOUNT_LEFT_IN,
        "mountYawDeg" to DEFAULT_MOUNT_YAW_DEG,
        "mountMeasured" to DEFAULT_MOUNT_MEASURED,
        "resetToDefaults" to DEFAULT_RESET_TO_DEFAULTS,
    )
}
