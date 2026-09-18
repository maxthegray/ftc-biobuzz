package org.firstinspires.ftc.teamcode.vision.diagnostics

import com.bylazar.configurables.annotations.Configurable

/**
 * Settings shared by both vision diagnostics, persisted under
 * `visionDiagnostics`. Every field is **restart**: it decides what hardware the
 * OpMode opens.
 */
@Configurable
object VisionDiagnosticsConfig {

    private const val DEFAULT_RUN_BOTH_CAMERAS = false
    private const val DEFAULT_LIMELIGHT_TAG_PIPELINE_INDEX = 1
    private const val DEFAULT_LIMELIGHT_POLL_RATE_HZ = 100
    private const val DEFAULT_LIMELIGHT_MAX_RESULT_AGE_MS = 100

    /** Open the Limelight and the USB camera together to compare timing under combined load. */
    @JvmField var runBothCameras: Boolean = DEFAULT_RUN_BOTH_CAMERAS

    /**
     * Limelight pipeline slot configured for AprilTags in the web interface.
     * Slot 0 is reserved for the archived Ball Follow color pipeline.
     */
    @JvmField var limelightTagPipelineIndex: Int = DEFAULT_LIMELIGHT_TAG_PIPELINE_INDEX

    @JvmField var limelightPollRateHz: Int = DEFAULT_LIMELIGHT_POLL_RATE_HZ

    /** Frames older than this since their first receipt are stale, even if duplicate polls keep arriving. */
    @JvmField var limelightMaxResultAgeMs: Int = DEFAULT_LIMELIGHT_MAX_RESULT_AGE_MS

    fun resetDefaults() {
        runBothCameras = DEFAULT_RUN_BOTH_CAMERAS
        limelightTagPipelineIndex = DEFAULT_LIMELIGHT_TAG_PIPELINE_INDEX
        limelightPollRateHz = DEFAULT_LIMELIGHT_POLL_RATE_HZ
        limelightMaxResultAgeMs = DEFAULT_LIMELIGHT_MAX_RESULT_AGE_MS
    }

    fun compiledDefaults(): Map<String, Any> = linkedMapOf(
        "runBothCameras" to DEFAULT_RUN_BOTH_CAMERAS,
        "limelightTagPipelineIndex" to DEFAULT_LIMELIGHT_TAG_PIPELINE_INDEX,
        "limelightPollRateHz" to DEFAULT_LIMELIGHT_POLL_RATE_HZ,
        "limelightMaxResultAgeMs" to DEFAULT_LIMELIGHT_MAX_RESULT_AGE_MS,
    )

    internal val safeTagPipelineIndex: Int
        get() = if (limelightTagPipelineIndex in 0..9) limelightTagPipelineIndex else DEFAULT_LIMELIGHT_TAG_PIPELINE_INDEX

    internal val safePollRateHz: Int
        get() = if (limelightPollRateHz in 1..250) limelightPollRateHz else DEFAULT_LIMELIGHT_POLL_RATE_HZ

    internal val safeMaxResultAgeMs: Long
        get() = (if (limelightMaxResultAgeMs > 0) limelightMaxResultAgeMs else DEFAULT_LIMELIGHT_MAX_RESULT_AGE_MS).toLong()
}
