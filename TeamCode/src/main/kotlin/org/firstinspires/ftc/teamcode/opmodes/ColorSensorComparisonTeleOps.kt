package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.core.hardware.SRSHub
import org.firstinspires.ftc.teamcode.core.hardware.SRSHubSubsystem
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.LoopPhase
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

@TeleOp(name = "Starter: Direct Color Sensor Test", group = "Starter")
class DirectColorSensorTestTeleOp : OpModeBase() {

    private lateinit var color: DirectColorSensorProbe

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(DIRECT_SENSOR_NAME, RevColorSensorV3::class.java))
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        color = robot.register(DirectColorSensorProbe(DIRECT_SENSOR_NAME))
    }

    override fun onStart() {
        robot.recordEvent("direct REV Color Sensor V3 comparison started")
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() {
        emitTelemetry()
        sleep(TEST_LOOP_DELAY_MS)
    }

    private fun emitTelemetry() {
        telemetryBag.section("Direct Color Sensor Test") {
            put("route", "Control Hub I2C -> $DIRECT_SENSOR_NAME")
            put("status", color.health())
            put("samples", color.sampleCount)
            put("read failures", color.readFailures)
            put("sample read ms", color.lastReadNanos / 1e6, decimals = 3)
            put("sample read max ms", color.maxReadNanos / 1e6, decimals = 3)
            put("periodic phase ms", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 3)
            put("red", color.red, decimals = 4)
            put("green", color.green, decimals = 4)
            put("blue", color.blue, decimals = 4)
            put("alpha", color.alpha, decimals = 4)
            put("proximity", color.proximity)
        }
        telemetryBag.section("Procedure") {
            put("1", "Run 30 seconds")
            put("2", "Show white, red, green, blue, then cover sensor")
            put("3", "Stop normally so the WPILOG closes")
        }
    }
}

@TeleOp(name = "Starter: SRS Color Sensor Test", group = "Starter")
class SrsColorSensorTestTeleOp : OpModeBase() {

    private lateinit var srsHub: SRSHubSubsystem
    private lateinit var color: SrsColorSensorProbe

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(SRS_HUB_NAME, SRSHub::class.java))
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        srsHub = robot.register(SRSHubSubsystem(SRS_HUB_NAME))
        color = robot.register(SrsColorSensorProbe(srsHub, srsHub.color(bus = SRS_COLOR_BUS)))
    }

    override fun onStart() {
        robot.recordEvent("SRS Hub REV Color Sensor V3 comparison started")
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() {
        emitTelemetry()
        sleep(TEST_LOOP_DELAY_MS)
    }

    private fun emitTelemetry() {
        telemetryBag.section("SRS Color Sensor Test") {
            put("route", "Control Hub I2C 3 -> $SRS_HUB_NAME -> SRS I2C $SRS_COLOR_BUS")
            put("hub", srsHub.health())
            put("sensor disconnected", color.disconnected)
            put("samples", color.sampleCount)
            put("periodic phase ms", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 3)
            put("red", color.red)
            put("green", color.green)
            put("blue", color.blue)
            put("infrared", color.infrared)
            put("proximity", color.proximity)
        }
        telemetryBag.section("Procedure") {
            put("1", "Run 30 seconds")
            put("2", "Show white, red, green, blue, then cover sensor")
            put("3", "Stop normally so the WPILOG closes")
        }
    }
}

@TeleOp(name = "Starter: Direct 3x Sensor Stress Test", group = "Starter")
class DirectThreeSensorStressTestTeleOp : OpModeBase() {

    private lateinit var colors: List<DirectColorSensorProbe>
    private lateinit var distance: DirectDistanceSensorProbe

    override val requiredDevices: List<Preflight.Requirement>
        get() = DIRECT_STRESS_COLOR_NAMES.map {
            Preflight.Requirement(it, RevColorSensorV3::class.java)
        } + Preflight.Requirement(DIRECT_STRESS_DISTANCE_NAME, Rev2mDistanceSensor::class.java)
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        colors = DIRECT_STRESS_COLOR_NAMES.mapIndexed { index, hardwareName ->
            robot.register(
                DirectColorSensorProbe(
                    hardwareName = hardwareName,
                    subsystemName = "DirectColor${index + 1}",
                ),
            )
        }
        distance = robot.register(DirectDistanceSensorProbe(DIRECT_STRESS_DISTANCE_NAME))
    }

    override fun onStart() {
        robot.recordEvent("direct 2x REV Color Sensor V3 + REV 2m distance stress test started")
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() {
        emitTelemetry()
        sleep(TEST_LOOP_DELAY_MS)
    }

    private fun emitTelemetry() {
        telemetryBag.section("Direct 3x Sensor Stress Test") {
            put("route", "two color + one distance directly on Control Hub I2C")
            put("periodic phase ms", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 3)
            put("combined failures", colors.sumOf { it.readFailures } + distance.readFailures)
        }
        colors.forEachIndexed { index, color ->
            telemetryBag.section("Direct ${DIRECT_STRESS_COLOR_NAMES[index]}") {
                put("status", color.health())
                put("samples", color.sampleCount)
                put("read ms", color.lastReadNanos / 1e6, decimals = 3)
                put("red", color.red, decimals = 4)
                put("green", color.green, decimals = 4)
                put("blue", color.blue, decimals = 4)
                put("proximity", color.proximity)
            }
        }
        telemetryBag.section("Direct $DIRECT_STRESS_DISTANCE_NAME") {
            put("status", distance.health())
            put("samples", distance.sampleCount)
            put("read ms", distance.lastReadNanos / 1e6, decimals = 3)
            put("distance mm", distance.distanceMm, decimals = 1)
        }
    }
}

@TeleOp(name = "Starter: SRS 3x Sensor Stress Test", group = "Starter")
class SrsThreeSensorStressTestTeleOp : OpModeBase() {

    private lateinit var srsHub: SRSHubSubsystem
    private lateinit var colors: List<SrsColorSensorProbe>
    private lateinit var distance: SrsDistanceSensorProbe

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(SRS_HUB_NAME, SRSHub::class.java))
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        srsHub = robot.register(SRSHubSubsystem(SRS_HUB_NAME))
        colors = SRS_STRESS_COLOR_BUSES.map { bus ->
            robot.register(
                SrsColorSensorProbe(
                    hub = srsHub,
                    sensor = srsHub.color(bus),
                    subsystemName = "SRSColor$bus",
                ),
            )
        }
        distance = robot.register(
            SrsDistanceSensorProbe(
                hub = srsHub,
                sensor = srsHub.distance(SRS_STRESS_DISTANCE_BUS),
            ),
        )
    }

    override fun onStart() {
        robot.recordEvent("SRS Hub 2x REV Color Sensor V3 + REV 2m distance stress test started")
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() {
        emitTelemetry()
        sleep(TEST_LOOP_DELAY_MS)
    }

    private fun emitTelemetry() {
        telemetryBag.section("SRS 3x Sensor Stress Test") {
            put("route", "Control Hub -> $SRS_HUB_NAME -> SRS I2C 1/2/3")
            put("hub", srsHub.health())
            put("periodic phase ms", robot.profile[LoopPhase.PERIODIC] / 1e6, decimals = 3)
            put(
                "disconnected sensors",
                colors.count { it.disconnected } + if (distance.disconnected) 1 else 0,
            )
        }
        colors.forEachIndexed { index, color ->
            telemetryBag.section("SRS I2C ${SRS_STRESS_COLOR_BUSES[index]}") {
                put("status", color.health())
                put("samples", color.sampleCount)
                put("red", color.red)
                put("green", color.green)
                put("blue", color.blue)
                put("infrared", color.infrared)
                put("proximity", color.proximity)
            }
        }
        telemetryBag.section("SRS I2C $SRS_STRESS_DISTANCE_BUS") {
            put("status", distance.health())
            put("samples", distance.sampleCount)
            put("distance mm", distance.distanceMm, decimals = 1)
        }
    }
}

private class DirectColorSensorProbe(
    private val hardwareName: String,
    subsystemName: String = "DirectColor",
) : SubsystemBase(subsystemName) {

    private lateinit var sensor: RevColorSensorV3

    var red = 0.0
        private set
    var green = 0.0
        private set
    var blue = 0.0
        private set
    var alpha = 0.0
        private set
    var proximity = 0L
        private set
    var sampleCount = 0L
        private set
    var readFailures = 0L
        private set
    var lastReadNanos = 0L
        private set
    var maxReadNanos = 0L
        private set
    private var lastError = "-"

    override fun init(hardwareMap: HardwareMap) {
        sensor = hardwareMap.get(RevColorSensorV3::class.java, hardwareName)
    }

    override fun periodic() {
        val startNs = System.nanoTime()
        try {
            val sample = sensor.normalizedColors
            red = sample.red.toDouble()
            green = sample.green.toDouble()
            blue = sample.blue.toDouble()
            alpha = sample.alpha.toDouble()
            proximity = sensor.rawOptical().toLong()
            sampleCount++
            lastError = "-"
        } catch (t: Throwable) {
            readFailures++
            lastError = "${t.javaClass.simpleName}: ${t.message}"
        } finally {
            lastReadNanos = System.nanoTime() - startNs
            if (lastReadNanos > maxReadNanos) maxReadNanos = lastReadNanos
        }
    }

    override fun health(): String =
        if (lastError == "-") "ready" else "read failed: $lastError"

    override fun logState(log: StateLog) {
        log.put("red", red)
        log.put("green", green)
        log.put("blue", blue)
        log.put("alpha", alpha)
        log.put("proximity", proximity)
        log.put("sampleCount", sampleCount)
        log.put("readFailures", readFailures)
        log.put("readNanos", lastReadNanos)
        log.put("maxReadNanos", maxReadNanos)
        log.put("status", health())
    }
}

private class SrsColorSensorProbe(
    private val hub: SRSHubSubsystem,
    private val sensor: SRSHubSubsystem.ColorHandle,
    subsystemName: String = "SRSColor",
) : SubsystemBase(subsystemName) {

    override val registerAfter: Class<out SubsystemBase> get() = SRSHubSubsystem::class.java

    var red = 0L
        private set
    var green = 0L
        private set
    var blue = 0L
        private set
    var infrared = 0L
        private set
    var proximity = 0L
        private set
    var disconnected = true
        private set
    var sampleCount = 0L
        private set

    override fun periodic() {
        red = sensor.red.toLong()
        green = sensor.green.toLong()
        blue = sensor.blue.toLong()
        infrared = sensor.ir.toLong()
        proximity = sensor.proximity.toLong()
        disconnected = sensor.disconnected
        sampleCount++
    }

    override fun health(): String = if (disconnected) "sensor disconnected" else "ready"

    override fun logState(log: StateLog) {
        log.put("red", red)
        log.put("green", green)
        log.put("blue", blue)
        log.put("infrared", infrared)
        log.put("proximity", proximity)
        log.put("disconnected", disconnected)
        log.put("sampleCount", sampleCount)
        log.put("hubHealth", hub.health() ?: "-")
    }
}

private class DirectDistanceSensorProbe(
    private val hardwareName: String,
    subsystemName: String = "DirectDistance",
) : SubsystemBase(subsystemName) {

    private lateinit var sensor: Rev2mDistanceSensor

    var distanceMm = 0.0
        private set
    var sampleCount = 0L
        private set
    var readFailures = 0L
        private set
    var lastReadNanos = 0L
        private set
    var maxReadNanos = 0L
        private set
    private var lastError = "-"

    override fun init(hardwareMap: HardwareMap) {
        sensor = hardwareMap.get(Rev2mDistanceSensor::class.java, hardwareName)
    }

    override fun periodic() {
        val startNs = System.nanoTime()
        try {
            distanceMm = sensor.getDistance(DistanceUnit.MM)
            sampleCount++
            lastError = "-"
        } catch (t: Throwable) {
            readFailures++
            lastError = "${t.javaClass.simpleName}: ${t.message}"
        } finally {
            lastReadNanos = System.nanoTime() - startNs
            if (lastReadNanos > maxReadNanos) maxReadNanos = lastReadNanos
        }
    }

    override fun health(): String =
        if (lastError == "-") "ready" else "read failed: $lastError"

    override fun logState(log: StateLog) {
        log.put("distanceMm", distanceMm)
        log.put("sampleCount", sampleCount)
        log.put("readFailures", readFailures)
        log.put("readNanos", lastReadNanos)
        log.put("maxReadNanos", maxReadNanos)
        log.put("status", health())
    }
}

private class SrsDistanceSensorProbe(
    private val hub: SRSHubSubsystem,
    private val sensor: SRSHubSubsystem.DistanceHandle,
    subsystemName: String = "SRSDistance",
) : SubsystemBase(subsystemName) {

    override val registerAfter: Class<out SubsystemBase> get() = SRSHubSubsystem::class.java

    var distanceMm = 0.0
        private set
    var disconnected = true
        private set
    var sampleCount = 0L
        private set

    override fun periodic() {
        distanceMm = sensor.distanceMm.toDouble()
        disconnected = sensor.disconnected
        sampleCount++
    }

    override fun health(): String = if (disconnected) "sensor disconnected" else "ready"

    override fun logState(log: StateLog) {
        log.put("distanceMm", distanceMm)
        log.put("disconnected", disconnected)
        log.put("sampleCount", sampleCount)
        log.put("hubHealth", hub.health() ?: "-")
    }
}

private const val DIRECT_SENSOR_NAME = "testColor"
private const val SRS_HUB_NAME = "srsHub"
private const val SRS_COLOR_BUS = 1
private const val TEST_LOOP_DELAY_MS = 20L
private val DIRECT_STRESS_COLOR_NAMES = listOf("testColor", "colorSensor2")
private const val DIRECT_STRESS_DISTANCE_NAME = "distance"
private val SRS_STRESS_COLOR_BUSES = listOf(1, 2)
private const val SRS_STRESS_DISTANCE_BUS = 3
