package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.hardware.SRSHub
import org.firstinspires.ftc.teamcode.core.hardware.SRSHubSubsystem
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.LoopPhase
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

@TeleOp(name = "SRS Loop Benchmark", group = "Diagnostics")
class SrsLoopBenchmarkTeleOp : OpModeBase() {

    private lateinit var hub: SRSHubSubsystem
    private lateinit var probe: SrsLoopBenchmarkProbe

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(SRS_HUB_NAME, SRSHub::class.java))
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        hub = robot.register(SRSHubSubsystem(SRS_HUB_NAME))
        val colors = COLOR_BUSES.map(hub::color)
        val distance = hub.distance(DISTANCE_BUS)
        probe = robot.register(SrsLoopBenchmarkProbe(hub, colors, distance))
    }

    override fun onStart() {
        robot.recordEvent("unpaced SRS loop benchmark started")
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() = emitTelemetry()

    private fun emitTelemetry() {
        telemetryBag.section("SRS Loop Benchmark") {
            put("route", "Control Hub I2C 3 -> $SRS_HUB_NAME -> SRS I2C 1/2/3")
            put("deliberate delay", "none")
            put("hub", hub.health())
            put("sensor status", probe.health())
            put("valid samples", probe.sampleCount)
            put("invalid ticks", probe.invalidTicks)
            put("periodic phase ms", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 3)
            put("last whole loop ms", robot.profile.totalNanos / 1e6, decimals = 3)
        }
        probe.colors.forEachIndexed { index, color ->
            telemetryBag.section("SRS I2C ${COLOR_BUSES[index]}") {
                put("red", color.red)
                put("green", color.green)
                put("blue", color.blue)
                put("infrared", color.infrared)
                put("proximity", color.proximity)
            }
        }
        telemetryBag.section("SRS I2C $DISTANCE_BUS") {
            put("distance mm", probe.distanceMm, decimals = 1)
        }
        telemetryBag.section("Procedure") {
            put("1", "Run about 2 minutes")
            put("2", "Leave sensors connected; this measures loop ceiling")
            put("3", "Stop normally so the WPILOG closes")
        }
    }
}

private class SrsLoopBenchmarkProbe(
    private val hub: SRSHubSubsystem,
    private val colorHandles: List<SRSHubSubsystem.ColorHandle>,
    private val distanceHandle: SRSHubSubsystem.DistanceHandle,
) : SubsystemBase("SRSBenchmark") {

    data class ColorSample(
        var red: Long = 0L,
        var green: Long = 0L,
        var blue: Long = 0L,
        var infrared: Long = 0L,
        var proximity: Long = 0L,
    )

    override val registerAfter: Class<out SubsystemBase> get() = SRSHubSubsystem::class.java

    val colors = List(colorHandles.size) { ColorSample() }
    var distanceMm = 0.0
        private set
    var sampleCount = 0L
        private set
    var invalidTicks = 0L
        private set

    override fun periodic() {
        if (!dataValid()) {
            invalidTicks++
            return
        }
        colorHandles.forEachIndexed { index, sensor ->
            colors[index].apply {
                red = sensor.red.toLong()
                green = sensor.green.toLong()
                blue = sensor.blue.toLong()
                infrared = sensor.ir.toLong()
                proximity = sensor.proximity.toLong()
            }
        }
        distanceMm = distanceHandle.distanceMm.toDouble()
        sampleCount++
    }

    override fun health(): String = when {
        hub.isDisconnected -> "hub disconnected; cached values invalid"
        !hub.isReady -> "hub not ready; cached values invalid"
        colorHandles.any { it.disconnected } || distanceHandle.disconnected ->
            "sensor disconnected; cached values invalid"
        else -> "ready"
    }

    override fun logState(log: StateLog) {
        log.put("hubReady", hub.isReady)
        log.put("hubDisconnected", hub.isDisconnected)
        log.put("status", health())
        log.put("sampleCount", sampleCount)
        log.put("invalidTicks", invalidTicks)
        colors.forEachIndexed { index, color ->
            val prefix = "color${index + 1}"
            log.put("$prefix/red", color.red)
            log.put("$prefix/green", color.green)
            log.put("$prefix/blue", color.blue)
            log.put("$prefix/infrared", color.infrared)
            log.put("$prefix/proximity", color.proximity)
            log.put("$prefix/disconnected", colorHandles[index].disconnected)
        }
        log.put("distance/distanceMm", distanceMm)
        log.put("distance/disconnected", distanceHandle.disconnected)
    }

    private fun dataValid(): Boolean =
        hub.isReady &&
            !hub.isDisconnected &&
            colorHandles.none { it.disconnected } &&
            !distanceHandle.disconnected
}

private const val SRS_HUB_NAME = "srsHub"
private val COLOR_BUSES = listOf(1, 2)
private const val DISTANCE_BUS = 3
