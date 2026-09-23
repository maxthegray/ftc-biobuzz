package org.firstinspires.ftc.teamcode.vision.hive

import com.bylazar.configurables.annotations.Configurable

/**
 * Where the Limelight sits on the turret, persisted under `turretCamera`.
 * Tag measurements are turned into robot coordinates with these values and
 * the turret angle at capture time, so goal height (and with it raised/lowered
 * HIVE state) is only as good as [heightIn] and [pitchUpDeg].
 *
 * Robot frame: x forward, y left, z up from the tiles, origin at the robot's
 * pose point. Turret angle 0 points the turret at the robot's front, positive
 * counterclockwise. For a bench survey the same fields describe a tripod with
 * the turret angle held at 0.
 *
 * The Limelight must be mounted upright (image not flipped); leave [measured]
 * false until the numbers come from a tape measure.
 */
@Configurable
object TurretCameraMountConfig {

    private const val DEFAULT_AXIS_FORWARD_IN = 0.0
    private const val DEFAULT_AXIS_LEFT_IN = 0.0
    private const val DEFAULT_FORWARD_IN = 0.0
    private const val DEFAULT_LEFT_IN = 0.0
    private const val DEFAULT_HEIGHT_IN = 0.0
    private const val DEFAULT_PITCH_UP_DEG = 0.0
    private const val DEFAULT_YAW_DEG = 0.0
    private const val DEFAULT_MEASURED = false

    /** Turret rotation axis from the robot's pose point: + toward the robot's front, inches. */
    @JvmField var axisForwardIn: Double = DEFAULT_AXIS_FORWARD_IN

    /** Turret rotation axis from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var axisLeftIn: Double = DEFAULT_AXIS_LEFT_IN

    /** Lens from the turret axis at turret angle 0: + toward the robot's front, inches. */
    @JvmField var forwardIn: Double = DEFAULT_FORWARD_IN

    /** Lens from the turret axis at turret angle 0: + toward the robot's left, inches. */
    @JvmField var leftIn: Double = DEFAULT_LEFT_IN

    /** Lens centre above the tiles, inches. */
    @JvmField var heightIn: Double = DEFAULT_HEIGHT_IN

    /** Optical axis above horizontal, degrees; positive tilts the camera up toward the tags. */
    @JvmField var pitchUpDeg: Double = DEFAULT_PITCH_UP_DEG

    /** Optical axis from the turret's forward direction, degrees, counterclockwise-positive. */
    @JvmField var yawDeg: Double = DEFAULT_YAW_DEG

    /** True once the values above describe the camera as mounted. */
    @JvmField var measured: Boolean = DEFAULT_MEASURED

    fun resetDefaults() {
        axisForwardIn = DEFAULT_AXIS_FORWARD_IN
        axisLeftIn = DEFAULT_AXIS_LEFT_IN
        forwardIn = DEFAULT_FORWARD_IN
        leftIn = DEFAULT_LEFT_IN
        heightIn = DEFAULT_HEIGHT_IN
        pitchUpDeg = DEFAULT_PITCH_UP_DEG
        yawDeg = DEFAULT_YAW_DEG
        measured = DEFAULT_MEASURED
    }

    fun compiledDefaults(): Map<String, Any> = linkedMapOf(
        "axisForwardIn" to DEFAULT_AXIS_FORWARD_IN,
        "axisLeftIn" to DEFAULT_AXIS_LEFT_IN,
        "forwardIn" to DEFAULT_FORWARD_IN,
        "leftIn" to DEFAULT_LEFT_IN,
        "heightIn" to DEFAULT_HEIGHT_IN,
        "pitchUpDeg" to DEFAULT_PITCH_UP_DEG,
        "yawDeg" to DEFAULT_YAW_DEG,
        "measured" to DEFAULT_MEASURED,
    )
}

/** Immutable copy of [TurretCameraMountConfig], taken once per tick. */
data class TurretCameraMount(
    val axisForwardIn: Double = 0.0,
    val axisLeftIn: Double = 0.0,
    val forwardIn: Double = 0.0,
    val leftIn: Double = 0.0,
    val heightIn: Double = 0.0,
    val pitchUpDeg: Double = 0.0,
    val yawDeg: Double = 0.0,
    val measured: Boolean = false,
) {
    companion object {
        fun fromConfig() = TurretCameraMount(
            axisForwardIn = TurretCameraMountConfig.axisForwardIn.finiteOr(0.0),
            axisLeftIn = TurretCameraMountConfig.axisLeftIn.finiteOr(0.0),
            forwardIn = TurretCameraMountConfig.forwardIn.finiteOr(0.0),
            leftIn = TurretCameraMountConfig.leftIn.finiteOr(0.0),
            heightIn = TurretCameraMountConfig.heightIn.finiteOr(0.0),
            pitchUpDeg = TurretCameraMountConfig.pitchUpDeg.finiteOr(0.0),
            yawDeg = TurretCameraMountConfig.yawDeg.finiteOr(0.0),
            measured = TurretCameraMountConfig.measured,
        )
    }
}

internal fun Double.finiteOr(fallback: Double): Double = if (isFinite()) this else fallback
