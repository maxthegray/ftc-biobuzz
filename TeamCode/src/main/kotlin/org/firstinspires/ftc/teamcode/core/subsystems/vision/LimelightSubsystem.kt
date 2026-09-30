package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.Clock
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

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
            val device = hardwareMap.tryGet(Limelight3A::class.java, hardwareName)
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

data class LimelightColorTarget(
    val txDegrees: Double,
    val tyDegrees: Double,
    val areaPercent: Double,
)

/**
 * A 6-DOF pose exactly as the Limelight reports it: metres and degrees, in
 * the frame named by the field that holds it. Limelight camera space is
 * +X right, +Y down, +Z out of the lens; target space is +X right (looking at
 * the tag), +Y down, +Z out of the tag face.
 */
data class LimelightPose(
    val xMeters: Double,
    val yMeters: Double,
    val zMeters: Double,
    val rollDegrees: Double,
    val pitchDegrees: Double,
    val yawDegrees: Double,
) {
    val distanceMeters: Double get() = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters)
}

/**
 * One decoded fiducial from a single Limelight result. Only camera-relative
 * measurements are carried: robot- and field-space poses depend on mounting
 * and field-map settings entered in the Limelight UI, which this code does not
 * own. The 3D poses are null when the pipeline did not solve them (Full 3D off
 * or no solution); the SDK substitutes an all-zero pose in that case.
 */
data class LimelightFiducial(
    val id: Int,
    val family: String,
    /** Degrees from the crosshair, positive right. */
    val txDegrees: Double,
    /** Degrees from the crosshair, sign as reported by the Limelight. */
    val tyDegrees: Double,
    /** Degrees from the principal pixel rather than the crosshair. */
    val txNoCrosshairDegrees: Double,
    val tyNoCrosshairDegrees: Double,
    /** Target area as reported by the SDK (documented 0–100 % of image). */
    val areaPercent: Double,
    val targetPoseCameraSpace: LimelightPose?,
    val cameraPoseTargetSpace: LimelightPose?,
    val cornerCount: Int,
)

data class LimelightReading(
    /** Control Hub wall clock (ms since epoch) when the SDK parsed this result. */
    val receiptTimestampMs: Long = 0L,
    /** Control Hub wall-clock age of [receiptTimestampMs], sampled when this reading was taken. */
    val ageMs: Long = Long.MAX_VALUE,
    val valid: Boolean = false,
    val pipelineIndex: Int = -1,
    val pipelineType: String = "",
    val txDegrees: Double = 0.0,
    val tyDegrees: Double = 0.0,
    val areaPercent: Double = 0.0,
    val captureLatencyMs: Double = 0.0,
    val targetingLatencyMs: Double = 0.0,
    val parseLatencyMs: Double = 0.0,
    /**
     * Limelight-local result timestamp (`ts`, ms since Limelight boot). Not
     * comparable with any Control Hub clock; used only as frame identity.
     * Zero when the device did not report one.
     */
    val limelightTimestampMs: Double = 0.0,
    val colorTargets: List<LimelightColorTarget> = emptyList(),
    val fiducials: List<LimelightFiducial> = emptyList(),
)

/** Minimal Limelight surface used by [LimelightSubsystem] and host tests. */
interface LimelightSource {
    val isRunning: Boolean
    val isConnected: Boolean

    fun setPollRateHz(rateHz: Int)
    fun pipelineSwitch(index: Int): Boolean
    fun start()
    fun latestReading(): LimelightReading
    fun stop()
}

internal class RealLimelightSource(private val limelight: Limelight3A) : LimelightSource {
    override val isRunning: Boolean get() = limelight.isRunning
    override val isConnected: Boolean get() = limelight.isConnected

    // The SDK replaces its result object once per poll; converting only new
    // objects keeps the robot loop from re-parsing a duplicate every tick.
    private var lastResult: LLResult? = null
    private var lastReading = LimelightReading()

    override fun setPollRateHz(rateHz: Int) = limelight.setPollRateHz(rateHz)

    override fun pipelineSwitch(index: Int): Boolean = limelight.pipelineSwitch(index)

    override fun start() = limelight.start()

    override fun latestReading(): LimelightReading {
        val result = limelight.latestResult
        if (result !== lastResult) {
            lastResult = result
            lastReading = convert(result)
        }
        return lastReading.copy(ageMs = result.staleness)
    }

    private fun convert(result: LLResult) = LimelightReading(
        receiptTimestampMs = result.controlHubTimeStamp,
        ageMs = result.staleness,
        valid = result.isValid,
        pipelineIndex = result.pipelineIndex,
        pipelineType = result.pipelineType,
        txDegrees = result.tx,
        tyDegrees = result.ty,
        areaPercent = result.ta,
        captureLatencyMs = result.captureLatency,
        targetingLatencyMs = result.targetingLatency,
        parseLatencyMs = result.parseLatency,
        limelightTimestampMs = result.timestamp,
        colorTargets = result.colorResults.map {
            LimelightColorTarget(
                txDegrees = it.targetXDegrees,
                tyDegrees = it.targetYDegrees,
                areaPercent = it.targetArea,
            )
        },
        fiducials = result.fiducialResults.map {
            LimelightFiducial(
                id = it.fiducialId,
                family = it.family,
                txDegrees = it.targetXDegrees,
                tyDegrees = it.targetYDegrees,
                txNoCrosshairDegrees = it.targetXDegreesNoCrosshair,
                tyNoCrosshairDegrees = it.targetYDegreesNoCrosshair,
                areaPercent = it.targetArea,
                targetPoseCameraSpace = it.targetPoseCameraSpace.toLimelightPose(),
                cameraPoseTargetSpace = it.cameraPoseTargetSpace.toLimelightPose(),
                cornerCount = it.targetCorners?.size ?: 0,
            )
        },
    )

    override fun stop() = limelight.stop()
}

private fun Pose3D?.toLimelightPose(): LimelightPose? {
    if (this == null) return null
    val p = position.toUnit(DistanceUnit.METER)
    val pose = LimelightPose(
        xMeters = p.x,
        yMeters = p.y,
        zMeters = p.z,
        rollDegrees = orientation.getRoll(AngleUnit.DEGREES),
        pitchDegrees = orientation.getPitch(AngleUnit.DEGREES),
        yawDegrees = orientation.getYaw(AngleUnit.DEGREES),
    )
    return pose.takeUnless { it.isUnsolved() }
}

internal fun LimelightPose.isUnsolved(): Boolean =
    xMeters == 0.0 && yMeters == 0.0 && zMeters == 0.0 &&
        rollDegrees == 0.0 && pitchDegrees == 0.0 && yawDegrees == 0.0
