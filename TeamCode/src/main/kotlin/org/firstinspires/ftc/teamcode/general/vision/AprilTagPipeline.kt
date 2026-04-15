package org.firstinspires.ftc.teamcode.general.vision

import com.pedropathing.ftc.InvertedFTCCoordinates
import com.pedropathing.ftc.PoseConverter
import com.pedropathing.geometry.PedroCoordinates
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

/**
 * Builds an [AprilTagProcessor] and converts SDK-computed robot poses into Pedro
 * field coordinates.
 *
 * The FTC SDK already does the camera-offset + tag-pose math for us: when you
 * pass [cameraPosition] / [cameraOrientation] via `setCameraPose()` and a
 * [tagLibrary] via `setTagLibrary()`, every detection comes back with a
 * `robotPose` populated in the FTC field frame. We just translate that frame
 * into Pedro's via [InvertedFTCCoordinates] (the official DECODE converter
 * shipped by Pedro 2.1.1: 90° rotation + (72, 72) translation).
 *
 * Earlier versions of this file did the math by hand from `ftcPose.x/y/yaw`.
 * That was wrong in two places (missing camera-heading rotation, and a
 * reflect-vs-rotate frame mismatch with Pedro), and reinvented work the SDK
 * already does correctly. Don't add it back.
 *
 * ## Camera pose convention (FTC, NOT Pedro)
 * - [cameraPosition]: lens position in the robot frame where +x = right,
 *   +y = forward, +z = up. Inches.
 * - [cameraOrientation]: yaw/pitch/roll of the camera relative to the robot,
 *   degrees. Yaw is rotation about +z (CCW positive) — 0 = pointing forward.
 *
 * ## Ignored tags
 * [ignoredTagIds] defaults to {21, 22, 23} (DECODE Obelisk motif tags). They
 * have no fixed field position, and the SDK library returns (0, 0, 0) for
 * them — letting them through would teleport the robot to field origin.
 */
class AprilTagPipeline(
    val cameraPosition: Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0),
    val cameraOrientation: YawPitchRollAngles = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0),
    val tagLibrary: AprilTagLibrary = AprilTagGameDatabase.getDecodeTagLibrary(),
    val ignoredTagIds: Set<Int> = setOf(21, 22, 23),
) {

    fun buildProcessor(): AprilTagProcessor =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setTagLibrary(tagLibrary)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build()

    /**
     * Convert one detection to a Pedro-frame robot pose. Returns null if the
     * tag id is on the ignore list, the SDK couldn't compute a robot pose
     * (unknown tag, lost-lock frame, missing intrinsics), or the detection
     * range is zero.
     */
    fun detectionToFieldPose(detection: AprilTagDetection): RobotPoseFromTag? {
        if (detection.id in ignoredTagIds) return null
        val robotPose = detection.robotPose ?: return null
        val ftcPose = detection.ftcPose ?: return null
        if (ftcPose.range <= 0.0) return null

        val pos = robotPose.position.toUnit(DistanceUnit.INCH)
        val yawRad = robotPose.orientation.getYaw(AngleUnit.RADIANS)

        val pedroPose = PoseConverter
            .pose2DToPose(
                Pose2D(DistanceUnit.INCH, pos.x, pos.y, AngleUnit.RADIANS, yawRad),
                InvertedFTCCoordinates.INSTANCE,
            )
            .getAsCoordinateSystem(PedroCoordinates.INSTANCE)

        return RobotPoseFromTag(
            fieldPose = Pose(pedroPose.x, pedroPose.y, pedroPose.heading),
            rangeInches = ftcPose.range,
            tagId = detection.id,
        )
    }

    data class RobotPoseFromTag(val fieldPose: Pose, val rangeInches: Double, val tagId: Int)
}
