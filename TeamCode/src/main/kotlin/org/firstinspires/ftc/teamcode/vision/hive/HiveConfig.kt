package org.firstinspires.ftc.teamcode.vision.hive

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.core.logging.SettingsChangeLog

/**
 * HIVE tracking settings, live-editable in Panels: where the Limelight sits on
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

    /** Turret rotation axis from the robot's pose point: + toward the robot's front, inches. */
    @JvmField var axisForwardIn = 0.0

    /** Turret rotation axis from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var axisLeftIn = 0.0

    /** Lens from the turret axis at turret angle 0: + toward the robot's front, inches. */
    @JvmField var cameraForwardIn = 0.0

    /** Lens from the turret axis at turret angle 0: + toward the robot's left, inches. */
    @JvmField var cameraLeftIn = 0.0

    /** Lens centre above the tiles, inches. */
    @JvmField var cameraHeightIn = 0.0

    /** Optical axis above horizontal, degrees; positive tilts the camera up toward the tags. */
    @JvmField var cameraPitchUpDeg = 0.0

    /** Optical axis from the turret's forward direction, degrees, counterclockwise-positive. */
    @JvmField var cameraYawDeg = 0.0

    /** True once the camera fields above come from a tape measure. */
    @JvmField var mountMeasured = false

    /** Tag-centre height above the tiles separating a raised CELL's tags from a lowered CELL's. */
    @JvmField var raisedMinHeightIn = 43.95

    /** Tags within this of [raisedMinHeightIn] are ambiguous (e.g. mid-tip) and cast no HIVE-state vote. */
    @JvmField var classificationMarginIn = 3.0

    /** Tags whose goal point is farther than this from the CELL's median are dropped from the fused goal. */
    @JvmField var maxTagSpreadIn = 2.0

    /** A goal not re-observed for this long is reported lost. */
    @JvmField var lostTimeoutMs = 500

    /** Consecutive agreeing frames required to change a HIVE's state. */
    @JvmField var tipConfirmFrames = 3

    /** Values as compiled, captured before anything can edit them; lab records flag what differs. */
    val compiledDefaults: Map<String, String> = SettingsChangeLog.valuesOf(this)
}
