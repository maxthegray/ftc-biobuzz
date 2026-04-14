package org.firstinspires.ftc.teamcode.starter.vision

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/**
 * Builds a configured [AprilTagProcessor] and converts detections into
 * field-centric robot poses usable by the starter's localisation stack.
 *
 * Two things are configurable:
 *
 *  1. Camera-on-robot offset: where is the camera lens relative to the
 *     robot origin (center of rotation), in inches and radians. This lets
 *     the same detection logic work whether the camera is front-center,
 *     up high, or behind the turret.
 *
 *  2. Tag library: a function `(tagId) -> fieldPose` supplied by the
 *     op-mode, since the real tag positions are game-specific and change
 *     every season. The default returns `null` for every id and so rejects
 *     every detection — you must provide this to use pose fusion.
 */
class AprilTagPipeline(
    val cameraOffset: Pose = Pose(0.0, 0.0, 0.0),
    val tagLibrary: (Int) -> Pose? = { null },
) {

    /** Build a pre-configured AprilTagProcessor ready to hand to a VisionPortal. */
    fun buildProcessor(): AprilTagProcessor =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)
            .build()

    /**
     * Compute a field-centric robot pose from a single [AprilTagDetection].
     * Returns null if the tag id is unknown, the pose data is missing, or
     * the detection's range is zero (lost-lock frame).
     *
     * This is a straightforward "tag known pose + camera-relative offset"
     * transform. It intentionally does NOT do any smoothing or reject-on-
     * jitter logic — that's the [org.firstinspires.ftc.teamcode.starter
     * .localization.AprilTagCorrector]'s job.
     */
    fun detectionToFieldPose(detection: AprilTagDetection): RobotPoseFromTag? {
        val tagField = tagLibrary(detection.id) ?: return null
        val ftcPose = detection.ftcPose ?: return null
        if (ftcPose.range <= 0.0) return null

        // ftcPose: x is right, y is forward, yaw is heading (radians) — all
        // in the camera's reference frame with units = inches/radians per
        // our Builder config.
        val camX = ftcPose.y       // forward becomes +X in robot frame
        val camY = -ftcPose.x      // right becomes -Y in robot frame
        val camYaw = ftcPose.yaw   // tag heading relative to camera

        // Transform camera-relative measurements into robot-relative, then
        // into field-relative using the tag's known field pose and the
        // inverse of the camera mount.
        val robotRelX = camX - cameraOffset.x
        val robotRelY = camY - cameraOffset.y
        val robotHeading = tagField.heading - camYaw - cameraOffset.heading

        val cos = Math.cos(robotHeading)
        val sin = Math.sin(robotHeading)
        val fieldX = tagField.x - (robotRelX * cos - robotRelY * sin)
        val fieldY = tagField.y - (robotRelX * sin + robotRelY * cos)

        return RobotPoseFromTag(
            fieldPose = Pose(fieldX, fieldY, robotHeading),
            rangeInches = ftcPose.range,
            tagId = detection.id,
        )
    }

    /** Output bundle from [detectionToFieldPose]. */
    data class RobotPoseFromTag(val fieldPose: Pose, val rangeInches: Double, val tagId: Int)
}
