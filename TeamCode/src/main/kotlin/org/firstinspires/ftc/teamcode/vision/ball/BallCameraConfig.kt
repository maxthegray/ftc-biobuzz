package org.firstinspires.ftc.teamcode.vision.ball

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.core.logging.SettingsChangeLog

/**
 * Panels-tunable settings for the USB ball camera. Tentative values from the
 * September 18, 2026 tuning session. Panels edits last until the app restarts
 * or a hot reload; a lab record lists what you changed, so copy those values here.
 *
 * Fields marked **init** are built into the SDK's ColorBlobLocatorProcessor
 * when the OpMode initializes: edit them in Panels, then stop and re-init.
 * The rest apply on the next robot loop.
 *
 * The `mount*` fields record where the lens sits on the robot. Nothing on the
 * robot uses them: [BallCameraSubsystem] logs them every tick under
 * `BallCamera/mount/…` so MaxScope can place detections on the field.
 */
@Configurable
object BallCameraConfig {
    const val COLOR_SPACE_YCRCB = 0
    const val COLOR_SPACE_HSV = 1

    /** **init** 0 = YCrCb (channels Y, Cr, Cb), 1 = HSV (OpenCV H 0–180, S, V). */
    @JvmField var colorSpace = COLOR_SPACE_YCRCB

    /** **init** Inclusive threshold per channel of [colorSpace], 0–255 (HSV hue 0–180). */
    @JvmField var channel0Min = 125
    @JvmField var channel0Max = 255
    @JvmField var channel1Min = 130
    @JvmField var channel1Max = 170
    @JvmField var channel2Min = 50
    @JvmField var channel2Max = 110

    /** **init** Search region as fractions of the frame, origin top-left: 0 ≤ left < right ≤ 1, 0 ≤ top < bottom ≤ 1. */
    @JvmField var roiLeft = 0.0
    @JvmField var roiTop = 0.0
    @JvmField var roiRight = 1.0
    @JvmField var roiBottom = 1.0

    /** **init** Kernel sizes in pixels; 0 disables. */
    @JvmField var blurKernelPx = 5
    @JvmField var erodeKernelPx = 3
    @JvmField var dilateKernelPx = 3

    /** **init** false = erode then dilate (removes speckles); true = dilate then erode (fills holes). */
    @JvmField var morphClosing = false

    /** **init** Ignore contours nested inside another contour (glare holes inside a ball). */
    @JvmField var externalContoursOnly = true

    /** **init** Manual exposure keeps the color threshold valid as the lighting changes; false leaves auto exposure on. */
    @JvmField var exposureManual = true
    @JvmField var exposureMicros = 6000

    /** Contour-area limits as a percentage of the full frame. */
    @JvmField var minAreaPercent = 0.05
    @JvmField var maxAreaPercent = 40.0

    /** 4π·area/perimeter²; 1.0 is a perfect circle, which a traced contour rarely reaches. */
    @JvmField var minCircularity = 1.0

    /** Long/short side of the minimum-area rectangle; 1.0 is square. */
    @JvmField var maxAspectRatio = 2.5

    /** Contour area / convex-hull area; low for merged or notched blobs. */
    @JvmField var minDensity = 0.7

    /** Detections clear when no new frame has arrived for this long. */
    @JvmField var maxObservationAgeMs = 150.0

    /** Robot Controller live view. Off saves CPU. */
    @JvmField var previewEnabled = true

    /** Lens centre above the floor, inches. */
    @JvmField var mountHeightIn = 0.0

    /** Optical axis below horizontal, degrees; positive tilts the camera down. */
    @JvmField var mountPitchDownDeg = 0.0

    /** Lens position from the robot's pose point (its centre): + toward the robot's front, inches. */
    @JvmField var mountForwardIn = 0.0

    /** Lens position from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var mountLeftIn = 0.0

    /** Optical axis from the robot's front, degrees, counterclockwise-positive like Pedro headings. */
    @JvmField var mountYawDeg = 0.0

    /** True once the mount values above describe the camera as mounted. */
    @JvmField var mountMeasured = false

    /** Values as compiled, captured before anything can edit them; lab records flag what differs. */
    val compiledDefaults: Map<String, String> = SettingsChangeLog.valuesOf(this)
}
