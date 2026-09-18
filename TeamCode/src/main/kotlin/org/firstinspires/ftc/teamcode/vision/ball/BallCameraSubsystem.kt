package org.firstinspires.ftc.teamcode.vision.ball

import org.firstinspires.ftc.teamcode.vision.logging.SettingsChangeLog
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * The camera, processor, and control worker behind [BallCameraSubsystem].
 * Implementations own threads; every method here must return promptly.
 */
interface BallCameraBackend {
    /** Human-readable device/stream state, e.g. the VisionPortal camera state. */
    val cameraState: String

    /** Frame rate as the camera library reports it (delivery into the pipeline), or NaN. */
    val libraryReportedFps: Double

    fun latestFrame(): BallFrameResult?
    fun lensReport(): LensReport
    fun publishDetectionSettings(settings: DetectionSettings, version: Long)
    fun requestCameraImage(request: CameraImageRequest)
    fun controlStatus(): CameraControlStatus
    val processorFaults: Long
    val lastProcessorError: String?

    /** Release the camera, waiting at most [timeoutMs]; returns false if release was still pending. */
    fun close(timeoutMs: Long): Boolean
}

fun interface BallCameraBackendFactory {
    fun open(
        hardwareMap: HardwareMap,
        hardwareName: String,
        stream: StreamSettings,
        detection: DetectionSettings,
    ): BallCameraBackend
}

/**
 * Read-only USB ball camera: opens the camera at init, keeps the camera thread
 * supplied with validated settings, and exposes one [BallObservation] per loop.
 * Commands nothing.
 *
 * Thread boundaries: Panels writes [BallVisionConfig] statics; [periodic]
 * snapshots them on the robot thread and hands immutable copies to the camera
 * thread (detection) and the control thread (exposure, white balance, gain,
 * live view). Results come back as immutable [BallFrameResult]s.
 */
class BallCameraSubsystem(
    val hardwareName: String = DEFAULT_HARDWARE_NAME,
    private val backendFactory: BallCameraBackendFactory = BallCameraBackendFactory { map, name, stream, detection ->
        VisionPortalBallCamera(map, name, stream, detection)
    },
    private val clock: Clock = Clock.SYSTEM,
    private val settingsSource: () -> BallVisionSettings = { BallVisionSettings.fromConfig() },
    private val supportedStreams: List<StreamSettings> = StreamSettings.GOBILDA_3122_0004_0001_MODES,
    /** Flight-log event sink, normally `robot::recordEvent`; receives the tuning values in force. */
    eventSink: (String) -> Unit = {},
    private val configValues: () -> Map<String, String> = { SettingsChangeLog.valuesOf(BallVisionConfig) },
) : SubsystemBase("BallCamera") {

    private val settingsLog = SettingsChangeLog("ballVision", clock, eventSink)
    private var backend: BallCameraBackend? = null
    private var lastDetection: DetectionSettings? = null
    private var lastCamera: CameraImageRequest? = null
    private var candidateColumns = CandidateColumns.EMPTY

    val tracker = BallObservationTracker(clock)

    /** Stream settings in force since init; changing them needs an OpMode restart. */
    var runningStream: StreamSettings? = null
        private set
    var requestedStream: StreamSettings? = null
        private set
    var settings: BallVisionSettings? = null
        private set
    var settingsVersion = 0L
        private set
    var closed = false
        private set
    var closeTimedOut = false
        private set

    val observation: BallObservation get() = tracker.observation
    val restartRequired: Boolean get() = runningStream != null && requestedStream != runningStream
    val cameraState: String get() = backend?.cameraState ?: if (closed) "CLOSED" else "NOT_OPENED"
    val libraryReportedFps: Double get() = backend?.libraryReportedFps ?: Double.NaN
    val lensReport: LensReport get() = backend?.lensReport() ?: LensReport.NOT_YET_KNOWN
    val controlStatus: CameraControlStatus get() = backend?.controlStatus() ?: CameraControlStatus()
    val processorFaults: Long get() = backend?.processorFaults ?: 0L
    val lastProcessorError: String? get() = backend?.lastProcessorError

    override fun init(hardwareMap: HardwareMap) {
        val initial = settingsSource()
        if (initial.stream !in supportedStreams) {
            throw HardwareConfigError(
                "Ball camera mode ${initial.stream} is not supported; use one of " +
                    supportedStreams.joinToString(", ") + " (ballVision.resolutionWidth/Height, streamFormat).",
            )
        }
        settings = initial
        settingsLog.update(configValues(), "detection v1")
        runningStream = initial.stream
        requestedStream = initial.stream
        settingsVersion = 1
        lastDetection = initial.detection
        val opened = backendFactory.open(hardwareMap, hardwareName, initial.stream, initial.detection)
        backend = opened
        opened.publishDetectionSettings(initial.detection, settingsVersion)
        lastCamera = initial.camera
        opened.requestCameraImage(initial.camera)
    }

    override fun periodic() {
        val active = backend ?: return
        if (BallVisionConfig.resetToDefaults) BallVisionConfig.resetDefaults()

        val previous = settings
        val current = settingsSource()
        settings = current
        requestedStream = current.stream
        if (current.detection != lastDetection) {
            lastDetection = current.detection
            settingsVersion++
            active.publishDetectionSettings(current.detection, settingsVersion)
        }
        // Reflection only when something changed or a merged edit is waiting to be written.
        if (current != previous || settingsLog.hasPending) {
            settingsLog.update(configValues(), "detection v$settingsVersion")
        }
        if (current.camera != lastCamera) {
            lastCamera = current.camera
            active.requestCameraImage(current.camera)
        }
        tracker.update(active.latestFrame(), current.maxObservationAgeMs)
    }

    override fun health(): String = when {
        closed -> "closed"
        backend == null -> "not opened"
        restartRequired -> "restart required for $requestedStream"
        else -> "${cameraState.lowercase(Locale.US)}; ${observation.frameStatus.name.lowercase(Locale.US)}; " +
            observation.targetStatus.name.lowercase(Locale.US)
    }

    override fun logState(log: StateLog) {
        val obs = observation
        val frame = obs.frame
        log.put("camera/state", cameraState)
        log.put("camera/libraryReportedFps", libraryReportedFps)
        log.put("camera/restartRequired", restartRequired)
        log.put("frame/number", frame?.frameNumber ?: 0L)
        log.put("frame/status", obs.frameStatus.name)
        log.put("frame/new", obs.newFrame)
        log.put("frame/ageMs", obs.ageMs)
        log.put("frame/captureTimeValid", obs.captureTimeValid)
        log.put("frame/captureToPublishMs", obs.captureToPublishMs)
        log.put("frame/processingMs", frame?.processingMs ?: Double.NaN)
        log.put("frame/publishToReceiptMs", obs.publishToReceiptMs)
        log.put("frame/settingsVersion", frame?.settingsVersion ?: 0L)
        log.put("frame/widthPx", (frame?.widthPx ?: 0).toLong())
        log.put("frame/heightPx", (frame?.heightPx ?: 0).toLong())
        log.put("rate/processedFps", tracker.processedFps)
        log.put("rate/receivedFps", tracker.receivedFps)
        log.put("rate/skippedFrames", tracker.skippedFrames)
        log.put("rate/repeatTicks", tracker.repeatTicks)
        log.put("faults/processor", processorFaults)
        log.put("candidates/contours", (frame?.contourCount ?: 0).toLong())
        log.put("candidates/evaluated", (frame?.selection?.evaluatedCount ?: 0).toLong())
        log.put("candidates/accepted", (frame?.selection?.acceptedCount ?: 0).toLong())
        val intrinsics = lensReport.intrinsics
        if (obs.candidates !== candidateColumns.candidates || obs.selected !== candidateColumns.selected ||
            intrinsics !== candidateColumns.intrinsics
        ) {
            candidateColumns = CandidateColumns(obs.candidates, obs.selected, intrinsics)
        }
        val columns = candidateColumns
        log.put("candidates/xPx", columns.xPx)
        log.put("candidates/yPx", columns.yPx)
        log.put("candidates/radiusPx", columns.radiusPx)
        log.put("candidates/areaPx", columns.areaPx)
        log.put("candidates/circularity", columns.circularity)
        log.put("candidates/horizontalDeg", columns.horizontalDeg)
        log.put("candidates/verticalDeg", columns.verticalDeg)
        log.put("candidates/rejections", columns.rejections)
        log.put("candidates/selectedIndex", columns.selectedIndex)
        log.put("target/status", obs.targetStatus.name)
        val blob = obs.selected?.blob
        log.put("target/xPx", blob?.centroidXPx ?: Double.NaN)
        log.put("target/yPx", blob?.centroidYPx ?: Double.NaN)
        log.put("target/radiusPx", blob?.enclosingRadiusPx ?: Double.NaN)
        log.put("target/areaPx", blob?.contourAreaPx ?: Double.NaN)
        log.put("target/circularity", blob?.circularity ?: Double.NaN)
        val angles = blob?.let { b -> lensReport.intrinsics?.rayAnglesDegrees(b.centroidXPx, b.centroidYPx) }
        log.put("target/horizontalDeg", angles?.first ?: Double.NaN)
        log.put("target/verticalDeg", angles?.second ?: Double.NaN)
        val control = controlStatus
        log.put("controls/state", control.state.name)
        log.put("controls/deviceWrites", control.deviceWrites)
        log.put("controls/requestedExposureMicros", control.requested?.exposureMicros ?: -1L)
        log.put("controls/readbackExposureMicros", control.readback?.exposureMicros ?: -1L)
        log.put("controls/readbackWhiteBalanceK", (control.readback?.whiteBalanceKelvin ?: -1).toLong())
        log.put("mount/measured", BallCameraMountConfig.measured)
        log.put("mount/heightIn", BallCameraMountConfig.heightIn)
        log.put("mount/pitchDownDeg", BallCameraMountConfig.pitchDownDeg)
        log.put("mount/forwardIn", BallCameraMountConfig.forwardIn)
        log.put("mount/leftIn", BallCameraMountConfig.leftIn)
        log.put("mount/yawDeg", BallCameraMountConfig.yawDeg)
    }

    /**
     * The published candidates of the current observation as parallel columns,
     * accepted first, rebuilt only when the observation or lens changes. Angles
     * follow [LensIntrinsics.rayAnglesDegrees] (vertical + below the axis) and
     * are NaN until the lens is known. [rejections] names each candidate's
     * first failed filter, or `accepted`.
     */
    private class CandidateColumns(
        val candidates: List<BallCandidate>,
        val selected: BallCandidate?,
        val intrinsics: LensIntrinsics?,
    ) {
        val xPx = DoubleArray(candidates.size) { candidates[it].blob.centroidXPx }
        val yPx = DoubleArray(candidates.size) { candidates[it].blob.centroidYPx }
        val radiusPx = DoubleArray(candidates.size) { candidates[it].blob.enclosingRadiusPx }
        val areaPx = DoubleArray(candidates.size) { candidates[it].blob.contourAreaPx }
        val circularity = DoubleArray(candidates.size) { candidates[it].blob.circularity }
        private val angles = candidates.map { intrinsics?.rayAnglesDegrees(it.blob.centroidXPx, it.blob.centroidYPx) }
        val horizontalDeg = DoubleArray(candidates.size) { angles[it]?.first ?: Double.NaN }
        val verticalDeg = DoubleArray(candidates.size) { angles[it]?.second ?: Double.NaN }
        val rejections = candidates.joinToString(",") { it.rejection?.label ?: "accepted" }
        val selectedIndex = candidates.indexOfFirst { it === selected }.toLong()

        companion object {
            val EMPTY = CandidateColumns(emptyList(), null, null)
        }
    }

    /** Releases the camera within [CLOSE_TIMEOUT_MS]; safe to call before init or twice. */
    override fun stop() {
        val active = backend ?: return
        backend = null
        closed = true
        try {
            closeTimedOut = !active.close(CLOSE_TIMEOUT_MS)
        } catch (_: Throwable) {
            // Robot.stop() must give every subsystem a chance to clean up.
        }
    }

    companion object {
        const val DEFAULT_HARDWARE_NAME = "ballCamera"
        const val CLOSE_TIMEOUT_MS = 1500L
    }
}
