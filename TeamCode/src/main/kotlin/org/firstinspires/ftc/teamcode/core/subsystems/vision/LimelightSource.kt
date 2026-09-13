package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D

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
