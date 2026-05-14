package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.localization.AprilTagCorrector

/**
 * Wraps an FTC [VisionPortal] + [AprilTagProcessor] and, optionally, pipes
 * every good detection into an [AprilTagCorrector] so localisation gets
 * bounded over time.
 *
 * Usage:
 *
 * ```kotlin
 * val vision = robot.register(VisionSubsystem(
 *     cameraName = "Webcam 1",
 *     pipeline = AprilTagPipeline(
 *         cameraOffset = Pose(7.0, 0.0, 0.0),
 *         tagLibrary = DecodeTagLibrary::lookup,
 *     ),
 *     corrector = aprilTagCorrector,
 * ))
 * ```
 *
 * The VisionPortal is created lazily in [init] so the webcam doesn't open
 * until the op-mode actually runs.
 */
class VisionSubsystem(
    val cameraName: String = "Webcam 1",
    val pipeline: AprilTagPipeline = AprilTagPipeline(),
    private val corrector: AprilTagCorrector? = null,
) : SubsystemBase("Vision") {

    private var portal: VisionPortal? = null
    private var processor: AprilTagProcessor? = null

    /** Last observed detections. Empty list when the camera is between frames. */
    var lastDetections: List<AprilTagDetection> = emptyList()
        private set

    /** Last Pedro-coordinate robot poses derived from AprilTag detections. */
    var lastFieldPoses: List<AprilTagPipeline.RobotPoseFromTag> = emptyList()
        private set

    override fun init(hardwareMap: HardwareMap) {
        val processor = pipeline.buildProcessor()
        this.processor = processor

        val webcam = hardwareMap.get(WebcamName::class.java, cameraName)
        this.portal = VisionPortal.Builder()
            .setCamera(webcam)
            .setCameraResolution(android.util.Size(640, 480))
            .enableLiveView(false)
            .addProcessor(processor)
            .build()
    }

    override fun periodic() {
        val p = processor ?: return
        val detections = p.detections ?: emptyList()
        lastDetections = detections
        lastFieldPoses = detections.mapNotNull { pipeline.detectionToFieldPose(it) }
        if (corrector == null) return
        for (fp in lastFieldPoses) {
            corrector.submit(
                AprilTagCorrector.Observation(
                    fieldPose = fp.fieldPose,
                    rangeInches = fp.rangeInches,
                    tagId = fp.tagId,
                ),
            )
        }
    }

    /** Pauses the streamer (camera frames stop) — useful to save CPU during auton. */
    fun pause() { portal?.stopStreaming() }

    /** Resumes the streamer after a [pause]. */
    fun resume() { portal?.resumeStreaming() }

    override fun stop() {
        portal?.close()
        portal = null
        processor = null
    }
}
