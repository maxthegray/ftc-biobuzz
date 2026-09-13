package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * Read-only Limelight state for commands, telemetry, and logging: color
 * targets for color pipelines, fiducials for AprilTag pipelines.
 *
 * Timing keeps clock domains separate: [resultAgeMs] is Control
 * Hub wall-clock time since the SDK received and parsed the result, and
 * [limelightTimestampMs] is the device's own clock, used only to tell frames
 * apart. [frameAgeMs] retains the first receipt age and advances on the robot's
 * monotonic clock, so repeated polls cannot renew a frame's freshness.
 * [estimatedCaptureAgeMs] adds capture and targeting latencies to that age;
 * it excludes HTTP transport and the wait before the first poll.
 */
class LimelightSubsystem(
    val hardwareName: String = DEFAULT_HARDWARE_NAME,
    val pipelineIndex: Int = DEFAULT_PIPELINE_INDEX,
    val pollRateHz: Int = DEFAULT_POLL_RATE_HZ,
    val maxResultAgeMs: Long = DEFAULT_MAX_RESULT_AGE_MS,
    source: LimelightSource? = null,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Limelight") {

    init {
        require(pipelineIndex in 0..9) { "pipeline index must be between 0 and 9" }
        require(pollRateHz in 1..250) { "poll rate must be between 1 and 250 Hz" }
        require(maxResultAgeMs > 0) { "maximum result age must be positive" }
    }

    private var injectedSource: LimelightSource? = source
    private lateinit var source: LimelightSource
    private var lastReceiptTimestampMs = Long.MIN_VALUE
    private var lastLimelightTimestampMs = Double.NaN
    private var frameFirstSeenNs: Long? = null
    private var frameInitialAgeMs = 0.0
    private var rateWindowStartNs = Long.MIN_VALUE
    private var framesInRateWindow = 0L

    var isRunning = false
        private set
    var isConnected = false
        private set
    var pipelineSwitchAccepted = false
        private set
    var activePipelineIndex = -1
        private set
    var pipelineType = ""
        private set
    var resultAgeMs = Long.MAX_VALUE
        private set
    /** Age since the first receipt of this frame; duplicate polls never reset it. */
    var frameAgeMs = Double.POSITIVE_INFINITY
        private set
    var resultFresh = false
        private set
    var targetVisible = false
        private set
    var primaryTarget: LimelightColorTarget? = null
        private set
    var colorTargets: List<LimelightColorTarget> = emptyList()
        private set
    var fiducials: List<LimelightFiducial> = emptyList()
        private set
    var limelightTimestampMs = 0.0
        private set
    var newFrameThisTick = false
        private set
    var captureLatencyMs = 0.0
        private set
    var targetingLatencyMs = 0.0
        private set
    var parseLatencyMs = 0.0
        private set
    var resultRateHz = 0.0
        private set
    var receivedFrameCount = 0L
        private set
    var staleTickCount = 0L
        private set

    val targetCount: Int get() = colorTargets.size
    val pipelineMatches: Boolean get() = activePipelineIndex == pipelineIndex
    val totalLatencyMs: Double get() = captureLatencyMs + targetingLatencyMs + parseLatencyMs
    val estimatedCaptureAgeMs: Double
        get() = frameAgeMs + captureLatencyMs + targetingLatencyMs

    override fun init(hardwareMap: HardwareMap) {
        source = injectedSource ?: run {
            val device = DeviceReaders.maybe(hardwareMap, hardwareName, Limelight3A::class.java)
                ?: throw HardwareConfigError(
                    "Missing Limelight3A named \"$hardwareName\" in active configuration.",
                )
            RealLimelightSource(device)
        }
        source.setPollRateHz(pollRateHz)
        pipelineSwitchAccepted = source.pipelineSwitch(pipelineIndex)
        source.start()
        isRunning = source.isRunning
    }

    override fun periodic() {
        isRunning = source.isRunning
        isConnected = source.isConnected

        val reading = source.latestReading()
        activePipelineIndex = reading.pipelineIndex
        pipelineType = reading.pipelineType
        resultAgeMs = reading.ageMs
        captureLatencyMs = reading.captureLatencyMs
        targetingLatencyMs = reading.targetingLatencyMs
        parseLatencyMs = reading.parseLatencyMs
        limelightTimestampMs = reading.limelightTimestampMs

        updateResultRate(reading)

        resultFresh = isConnected && resultAgeMs >= 0 && frameAgeMs < maxResultAgeMs
        if (isConnected && !resultFresh) staleTickCount++

        targetVisible = resultFresh && pipelineMatches && reading.valid
        if (targetVisible) {
            primaryTarget = LimelightColorTarget(
                txDegrees = reading.txDegrees,
                tyDegrees = reading.tyDegrees,
                areaPercent = reading.areaPercent,
            )
            colorTargets = reading.colorTargets
            fiducials = reading.fiducials
        } else {
            primaryTarget = null
            colorTargets = emptyList()
            fiducials = emptyList()
        }
    }

    override fun health(): String = when {
        !isRunning -> "polling stopped"
        !isConnected -> "disconnected"
        !pipelineMatches && !pipelineSwitchAccepted ->
            "pipeline switch failed; $activePipelineIndex active, expected $pipelineIndex"
        !pipelineMatches -> "pipeline $activePipelineIndex active; expected $pipelineIndex"
        !resultFresh -> "stale frame ($frameAgeMs ms; receipt $resultAgeMs ms)"
        targetVisible && fiducials.isNotEmpty() -> "tracking ${fiducials.size} tag(s)"
        targetVisible -> "tracking $targetCount target(s)"
        else -> "ready; no target"
    }

    override fun logState(log: StateLog) {
        log.put("running", isRunning)
        log.put("connected", isConnected)
        log.put("pipeline/switchAccepted", pipelineSwitchAccepted)
        log.put("pipeline/expectedIndex", pipelineIndex.toLong())
        log.put("pipeline/activeIndex", activePipelineIndex.toLong())
        log.put("pipeline/type", pipelineType)
        log.put("result/fresh", resultFresh)
        log.put("result/ageMs", resultAgeMs)
        log.put("result/frameAgeMs", frameAgeMs)
        log.put("result/rateHz", resultRateHz)
        log.put("result/receivedFrames", receivedFrameCount)
        log.put("result/staleTicks", staleTickCount)
        log.put("result/limelightTimestampMs", limelightTimestampMs)
        log.put("result/estimatedCaptureAgeMs", estimatedCaptureAgeMs)
        log.put("latency/captureMs", captureLatencyMs)
        log.put("latency/targetingMs", targetingLatencyMs)
        log.put("latency/parseMs", parseLatencyMs)
        log.put("target/visible", targetVisible)
        log.put("target/count", targetCount.toLong())
        val target = primaryTarget
        log.put("target/txDegrees", target?.txDegrees ?: 0.0)
        log.put("target/tyDegrees", target?.tyDegrees ?: 0.0)
        log.put("target/areaPercent", target?.areaPercent ?: 0.0)
        log.put("fiducial/count", fiducials.size.toLong())
        log.put("fiducial/ids", fiducials.joinToString(",") { it.id.toString() })
    }

    override fun stop() {
        if (!::source.isInitialized) return
        try {
            source.stop()
        } catch (_: Throwable) {
            // Robot.stop() must give every subsystem a chance to clean up.
        } finally {
            isRunning = false
        }
    }

    /**
     * A poll can return the same device frame twice, so frames are identified by
     * the Limelight's own timestamp when it reports one, else by receipt time.
     */
    private fun updateResultRate(reading: LimelightReading) {
        val now = clock.nanos()
        if (rateWindowStartNs == Long.MIN_VALUE) rateWindowStartNs = now

        val deviceTs = reading.limelightTimestampMs
        val isNewFrame = if (deviceTs.isFinite() && deviceTs > 0.0) {
            deviceTs != lastLimelightTimestampMs
        } else {
            reading.receiptTimestampMs != lastReceiptTimestampMs
        }
        newFrameThisTick = isConnected && isNewFrame
        if (newFrameThisTick) {
            lastLimelightTimestampMs = deviceTs
            lastReceiptTimestampMs = reading.receiptTimestampMs
            frameFirstSeenNs = now
            frameInitialAgeMs = reading.ageMs.coerceAtLeast(0).toDouble()
            receivedFrameCount++
            framesInRateWindow++
        }
        frameAgeMs = frameFirstSeenNs?.let {
            maxOf(frameInitialAgeMs + (now - it) / 1e6, reading.ageMs.toDouble())
        } ?: Double.POSITIVE_INFINITY

        val elapsedNs = now - rateWindowStartNs
        if (elapsedNs >= RATE_WINDOW_NS) {
            resultRateHz = framesInRateWindow * 1e9 / elapsedNs
            framesInRateWindow = 0L
            rateWindowStartNs = now
        }
    }

    companion object {
        const val DEFAULT_HARDWARE_NAME = "limelight"
        const val DEFAULT_PIPELINE_INDEX = 0
        const val DEFAULT_POLL_RATE_HZ = 100
        const val DEFAULT_MAX_RESULT_AGE_MS = 100L
        private const val RATE_WINDOW_NS = 1_000_000_000L
    }
}
