package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem

@TeleOp(name = "Starter: Limelight Ball Test", group = "Starter Diagnostics")
class LimelightBallTestTeleOp : OpModeBase() {

    private lateinit var limelight: LimelightSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(
            Preflight.Requirement(
                LimelightSubsystem.DEFAULT_HARDWARE_NAME,
                Limelight3A::class.java,
            ),
        )
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        limelight = robot.register(LimelightSubsystem())
    }

    override fun onStart() {
        robot.recordEvent("Limelight ball test started")
    }

    override fun onInitLoop() = emitVisionTelemetry()

    override fun onLoop() = emitVisionTelemetry()

    private fun emitVisionTelemetry() {
        telemetryBag.section("Limelight") {
            put("hardware", limelight.hardwareName)
            put("state", limelight.health())
            put("polling", limelight.isRunning)
            put("connected", limelight.isConnected)
            put("pipeline", "${limelight.activePipelineIndex} (${limelight.pipelineType})")
            put("expected pipeline", limelight.pipelineIndex)
            put("pipeline switch accepted", limelight.pipelineSwitchAccepted)
            put("result fresh", limelight.resultFresh)
            put("result age ms", limelight.resultAgeMs)
            put("result rate Hz", limelight.resultRateHz, decimals = 1)
            put("received frames", limelight.receivedFrameCount)
            put("stale ticks", limelight.staleTickCount)
        }
        telemetryBag.section("Color Targets") {
            put("visible", limelight.targetVisible)
            put("count", limelight.targetCount)
            val target = limelight.primaryTarget
            put("primary tx deg", target?.txDegrees ?: "—")
            put("primary ty deg", target?.tyDegrees ?: "—")
            put("primary area %", target?.areaPercent ?: "—")
        }
        telemetryBag.section("Vision Latency") {
            put("capture ms", limelight.captureLatencyMs, decimals = 2)
            put("targeting ms", limelight.targetingLatencyMs, decimals = 2)
            put("parse ms", limelight.parseLatencyMs, decimals = 2)
            put("total ms", limelight.totalLatencyMs, decimals = 2)
        }
        telemetryBag.section("Procedure") {
            put("1", "Wait for state: ready or tracking")
            put("2", "Move one ball left/right; tx should cross zero")
            put("3", "Run 2 minutes and watch the Loop maxima")
            put("4", "Stop normally so the WPILOG closes")
        }
    }
}
