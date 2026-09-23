package org.firstinspires.ftc.teamcode.vision.hive

import com.bylazar.configurables.annotations.Configurable

/**
 * HIVE tracking settings, persisted under `hive`: where the Limelight sits on
 * the turret, and the thresholds for HIVE state and goal fusion. Goal geometry
 * itself is FIRST's (see `BiobuzzAprilTags`) and is not tunable.
 *
 * Robot frame: x forward, y left, z up from the tiles, origin at the robot's
 * pose point. Turret angle 0 points the turret at the robot's front, positive
 * counterclockwise. For a bench survey the camera fields describe a tripod
 * with the turret angle held at 0. The Limelight must be mounted upright
 * (image not flipped).
 *
 * Goal height, and with it raised/lowered HIVE state, is only as good as
 * [cameraHeightIn] and [cameraPitchUpDeg]. [raisedMinHeightIn] defaults to the
 * pivot height (manual §9.6.1, 43.95 in.); set it from the Hive Tag Survey to
 * the midpoint of the tag heights it reports for both states.
 */
@Configurable
object HiveConfig {

    private const val DEFAULT_AXIS_FORWARD_IN = 0.0
    private const val DEFAULT_AXIS_LEFT_IN = 0.0
    private const val DEFAULT_CAMERA_FORWARD_IN = 0.0
    private const val DEFAULT_CAMERA_LEFT_IN = 0.0
    private const val DEFAULT_CAMERA_HEIGHT_IN = 0.0
    private const val DEFAULT_CAMERA_PITCH_UP_DEG = 0.0
    private const val DEFAULT_CAMERA_YAW_DEG = 0.0
    private const val DEFAULT_MOUNT_MEASURED = false
    private const val DEFAULT_RAISED_MIN_HEIGHT_IN = 43.95
    private const val DEFAULT_CLASSIFICATION_MARGIN_IN = 3.0
    private const val DEFAULT_MAX_TAG_SPREAD_IN = 2.0
    private const val DEFAULT_LOST_TIMEOUT_MS = 500
    private const val DEFAULT_TIP_CONFIRM_FRAMES = 3

    /** Turret rotation axis from the robot's pose point: + toward the robot's front, inches. */
    @JvmField var axisForwardIn: Double = DEFAULT_AXIS_FORWARD_IN

    /** Turret rotation axis from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var axisLeftIn: Double = DEFAULT_AXIS_LEFT_IN

    /** Lens from the turret axis at turret angle 0: + toward the robot's front, inches. */
    @JvmField var cameraForwardIn: Double = DEFAULT_CAMERA_FORWARD_IN

    /** Lens from the turret axis at turret angle 0: + toward the robot's left, inches. */
    @JvmField var cameraLeftIn: Double = DEFAULT_CAMERA_LEFT_IN

    /** Lens centre above the tiles, inches. */
    @JvmField var cameraHeightIn: Double = DEFAULT_CAMERA_HEIGHT_IN

    /** Optical axis above horizontal, degrees; positive tilts the camera up toward the tags. */
    @JvmField var cameraPitchUpDeg: Double = DEFAULT_CAMERA_PITCH_UP_DEG

    /** Optical axis from the turret's forward direction, degrees, counterclockwise-positive. */
    @JvmField var cameraYawDeg: Double = DEFAULT_CAMERA_YAW_DEG

    /** True once the camera fields above come from a tape measure. */
    @JvmField var mountMeasured: Boolean = DEFAULT_MOUNT_MEASURED

    /** Tag-centre height above the tiles separating a raised CELL's tags from a lowered CELL's. */
    @JvmField var raisedMinHeightIn: Double = DEFAULT_RAISED_MIN_HEIGHT_IN

    /** Tags within this of [raisedMinHeightIn] are ambiguous (e.g. mid-tip) and cast no HIVE-state vote. */
    @JvmField var classificationMarginIn: Double = DEFAULT_CLASSIFICATION_MARGIN_IN

    /** Tags whose goal point is farther than this from the CELL's median are dropped from the fused goal. */
    @JvmField var maxTagSpreadIn: Double = DEFAULT_MAX_TAG_SPREAD_IN

    /** A goal not re-observed for this long is reported lost. */
    @JvmField var lostTimeoutMs: Int = DEFAULT_LOST_TIMEOUT_MS

    /** Consecutive agreeing frames required to change a HIVE's state. */
    @JvmField var tipConfirmFrames: Int = DEFAULT_TIP_CONFIRM_FRAMES

    fun resetDefaults() {
        axisForwardIn = DEFAULT_AXIS_FORWARD_IN
        axisLeftIn = DEFAULT_AXIS_LEFT_IN
        cameraForwardIn = DEFAULT_CAMERA_FORWARD_IN
        cameraLeftIn = DEFAULT_CAMERA_LEFT_IN
        cameraHeightIn = DEFAULT_CAMERA_HEIGHT_IN
        cameraPitchUpDeg = DEFAULT_CAMERA_PITCH_UP_DEG
        cameraYawDeg = DEFAULT_CAMERA_YAW_DEG
        mountMeasured = DEFAULT_MOUNT_MEASURED
        raisedMinHeightIn = DEFAULT_RAISED_MIN_HEIGHT_IN
        classificationMarginIn = DEFAULT_CLASSIFICATION_MARGIN_IN
        maxTagSpreadIn = DEFAULT_MAX_TAG_SPREAD_IN
        lostTimeoutMs = DEFAULT_LOST_TIMEOUT_MS
        tipConfirmFrames = DEFAULT_TIP_CONFIRM_FRAMES
    }

    fun compiledDefaults(): Map<String, Any> = linkedMapOf(
        "axisForwardIn" to DEFAULT_AXIS_FORWARD_IN,
        "axisLeftIn" to DEFAULT_AXIS_LEFT_IN,
        "cameraForwardIn" to DEFAULT_CAMERA_FORWARD_IN,
        "cameraLeftIn" to DEFAULT_CAMERA_LEFT_IN,
        "cameraHeightIn" to DEFAULT_CAMERA_HEIGHT_IN,
        "cameraPitchUpDeg" to DEFAULT_CAMERA_PITCH_UP_DEG,
        "cameraYawDeg" to DEFAULT_CAMERA_YAW_DEG,
        "mountMeasured" to DEFAULT_MOUNT_MEASURED,
        "raisedMinHeightIn" to DEFAULT_RAISED_MIN_HEIGHT_IN,
        "classificationMarginIn" to DEFAULT_CLASSIFICATION_MARGIN_IN,
        "maxTagSpreadIn" to DEFAULT_MAX_TAG_SPREAD_IN,
        "lostTimeoutMs" to DEFAULT_LOST_TIMEOUT_MS,
        "tipConfirmFrames" to DEFAULT_TIP_CONFIRM_FRAMES,
    )
}
