package org.firstinspires.ftc.teamcode.vision

import com.bylazar.configurables.annotations.Configurable

/**
 * Where the USB ball camera sits on the robot. Nothing on the robot uses it:
 * [BallCameraSubsystem] logs it every tick under `BallCamera/mount/…`, so each
 * flight log carries the geometry it was recorded with and MaxScope can place
 * detections on the field. Moving the camera only needs these values updated.
 *
 * Leave [measured] false until the numbers come from a tape measure; the
 * viewer then shows directions only.
 */
@Configurable
object BallCameraMountConfig {

    private const val DEFAULT_HEIGHT_IN = 0.0
    private const val DEFAULT_PITCH_DOWN_DEG = 0.0
    private const val DEFAULT_FORWARD_IN = 0.0
    private const val DEFAULT_LEFT_IN = 0.0
    private const val DEFAULT_YAW_DEG = 0.0
    private const val DEFAULT_MEASURED = false

    /** Lens centre above the floor, inches. */
    @JvmField var heightIn: Double = DEFAULT_HEIGHT_IN

    /** Optical axis below horizontal, degrees; positive tilts the camera down. */
    @JvmField var pitchDownDeg: Double = DEFAULT_PITCH_DOWN_DEG

    /** Lens position from the robot's pose point (its centre): + toward the robot's front, inches. */
    @JvmField var forwardIn: Double = DEFAULT_FORWARD_IN

    /** Lens position from the robot's pose point: + toward the robot's left, inches. */
    @JvmField var leftIn: Double = DEFAULT_LEFT_IN

    /** Optical axis from the robot's front, degrees, counterclockwise-positive like Pedro headings. */
    @JvmField var yawDeg: Double = DEFAULT_YAW_DEG

    /** True once the values above describe the camera as mounted. */
    @JvmField var measured: Boolean = DEFAULT_MEASURED

    fun resetDefaults() {
        heightIn = DEFAULT_HEIGHT_IN
        pitchDownDeg = DEFAULT_PITCH_DOWN_DEG
        forwardIn = DEFAULT_FORWARD_IN
        leftIn = DEFAULT_LEFT_IN
        yawDeg = DEFAULT_YAW_DEG
        measured = DEFAULT_MEASURED
    }
}
