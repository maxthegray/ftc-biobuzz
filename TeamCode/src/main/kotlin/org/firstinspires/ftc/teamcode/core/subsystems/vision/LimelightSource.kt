package org.firstinspires.ftc.teamcode.core.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A

data class LimelightColorTarget(
    val txDegrees: Double,
    val tyDegrees: Double,
    val areaPercent: Double,
)

data class LimelightReading(
    val receiptTimestampMs: Long = 0L,
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
    val colorTargets: List<LimelightColorTarget> = emptyList(),
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

    override fun setPollRateHz(rateHz: Int) = limelight.setPollRateHz(rateHz)

    override fun pipelineSwitch(index: Int): Boolean = limelight.pipelineSwitch(index)

    override fun start() = limelight.start()

    override fun latestReading(): LimelightReading {
        val result = limelight.latestResult
        return LimelightReading(
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
            colorTargets = result.colorResults.map {
                LimelightColorTarget(
                    txDegrees = it.targetXDegrees,
                    tyDegrees = it.targetYDegrees,
                    areaPercent = it.targetArea,
                )
            },
        )
    }

    override fun stop() = limelight.stop()
}
