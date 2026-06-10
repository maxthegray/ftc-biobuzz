package org.firstinspires.ftc.teamcode.core.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.HardwareConfigError
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.localization.PinpointSource

/**
 * Subsystem that owns an SRSHub and bulk-reads every sensor attached to it
 * in a single I2C transaction per tick.
 *
 * Usage from `Robot.configure { ... }`:
 *
 * ```kotlin
 * val srsHub = register(SRSHubSubsystem())
 * val intakeColor   = srsHub.color(bus = 1)
 * val liftEncoder   = srsHub.encoder(port = 1, type = SRSHub.Encoder.QUADRATURE)
 * val limitSwitch   = srsHub.digital(pin = 1)
 * val potentiometer = srsHub.analog(pin = 2)
 * val pinpoint      = srsHub.pinpoint(
 *     bus = 1,
 *     xPodOffsetMm = 84f, yPodOffsetMm = -168f,
 *     ticksPerMm = 13.26291192f,
 *     xDir = SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
 *     yDir = SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED,
 * )
 * ```
 *
 * Handles are stored by the caller; reads after [periodic] return whatever
 * the most recent `hub.update()` decoded. The hub itself is configured once
 * during [init] from the registrations made before init.
 */
class SRSHubSubsystem(name: String = "srsHub") : SubsystemBase(name) {

    private lateinit var hub: SRSHub
    private val config = SRSHub.Config()

    private val analogPins = mutableListOf<Int>()
    private val digitalPins = mutableListOf<Int>()
    private val encoders = mutableListOf<Int>()
    private val i2cDevices = mutableListOf<SRSHub.I2CDevice>()

    private var configured = false

    val isReady: Boolean get() = ::hub.isInitialized && hub.ready()
    val isDisconnected: Boolean get() = ::hub.isInitialized && hub.disconnected()

    fun analog(pin: Int): AnalogHandle {
        requireUnconfigured()
        config.setAnalogDigitalDevice(pin, SRSHub.AnalogDigitalDevice.ANALOG)
        analogPins += pin
        return AnalogHandle { hub.readAnalogDigitalDevice(pin) }
    }

    fun digital(pin: Int): DigitalHandle {
        requireUnconfigured()
        config.setAnalogDigitalDevice(pin, SRSHub.AnalogDigitalDevice.DIGITAL)
        digitalPins += pin
        return DigitalHandle { hub.readAnalogDigitalDevice(pin) != 0.0 }
    }

    fun encoder(port: Int, type: SRSHub.Encoder = SRSHub.Encoder.QUADRATURE): EncoderHandle {
        requireUnconfigured()
        config.setEncoder(port, type)
        encoders += port
        return EncoderHandle { hub.readEncoder(port) }
    }

    fun color(bus: Int): ColorHandle {
        val dev = SRSHub.APDS9151()
        register(bus, dev)
        return ColorHandle(dev)
    }

    fun distance(bus: Int): DistanceHandle {
        val dev = SRSHub.VL53L0X()
        register(bus, dev)
        return DistanceHandle(dev)
    }

    fun tofGrid(bus: Int, resolution: SRSHub.VL53L5CX.Resolution): TofGridHandle {
        val dev = SRSHub.VL53L5CX(resolution)
        register(bus, dev)
        return TofGridHandle(dev)
    }

    fun pinpoint(
        bus: Int,
        xPodOffsetMm: Float,
        yPodOffsetMm: Float,
        ticksPerMm: Float,
        xDir: SRSHub.GoBildaPinpoint.EncoderDirection,
        yDir: SRSHub.GoBildaPinpoint.EncoderDirection,
    ): PinpointHandle {
        val dev = SRSHub.GoBildaPinpoint(xPodOffsetMm, yPodOffsetMm, ticksPerMm, xDir, yDir)
        register(bus, dev)
        return PinpointHandle(bus, dev) { cmd -> hub.runCommand(cmd) }
    }

    /** Forward a write-side command (e.g. Pinpoint reset). */
    fun send(command: SRSHub.Command) {
        hub.runCommand(command)
    }

    override fun init(hardwareMap: HardwareMap) {
        hub = try {
            hardwareMap.get(SRSHub::class.java, name)
        } catch (t: Throwable) {
            throw HardwareConfigError(
                "Missing SRSHub named \"$name\" in active configuration.", t,
            )
        }
        hub.init(config)
        configured = true
    }

    override fun periodic() {
        if (::hub.isInitialized && hub.ready()) hub.update()
    }

    override fun health(): String = when {
        !::hub.isInitialized -> "not initialized"
        hub.disconnected() -> "disconnected"
        hub.ready() -> "ready"
        else -> "not ready"
    }

    private fun register(bus: Int, device: SRSHub.I2CDevice) {
        requireUnconfigured()
        config.addI2CDevice(bus, device)
        i2cDevices += device
    }

    private fun requireUnconfigured() {
        // The SRSHub.Config locks itself on hub.init(); attempts to mutate it
        // after that throw a non-obvious error from inside the driver.
        check(!configured) { "SRSHubSubsystem is already initialised; register devices before init()." }
    }

    class AnalogHandle internal constructor(private val read: () -> Double) {
        val voltage: Double get() = read()
    }

    class DigitalHandle internal constructor(private val read: () -> Boolean) {
        val state: Boolean get() = read()
    }

    class EncoderHandle internal constructor(private val read: () -> SRSHub.PosVel) {
        val position: Int get() = read().position
        val velocity: Int get() = read().velocity
    }

    class ColorHandle internal constructor(private val dev: SRSHub.APDS9151) {
        val red: Int get() = dev.red
        val green: Int get() = dev.green
        val blue: Int get() = dev.blue
        val ir: Int get() = dev.infrared
        val proximity: Short get() = dev.proximity
        val disconnected: Boolean get() = dev.disconnected
    }

    class DistanceHandle internal constructor(private val dev: SRSHub.VL53L0X) {
        val distanceMm: Float get() = dev.distance
        val disconnected: Boolean get() = dev.disconnected
    }

    class TofGridHandle internal constructor(private val dev: SRSHub.VL53L5CX) {
        val distances: ShortArray get() = dev.distances
        val disconnected: Boolean get() = dev.disconnected
    }

    class PinpointHandle internal constructor(
        val bus: Int,
        private val dev: SRSHub.GoBildaPinpoint,
        private val send: (SRSHub.Command) -> Unit,
    ) : PinpointSource {
        override val xMm: Float get() = dev.xPosition
        override val yMm: Float get() = dev.yPosition
        override val headingRad: Float get() = dev.hOrientation
        override val xVelMmPerSec: Float get() = dev.xVelocity
        override val yVelMmPerSec: Float get() = dev.yVelocity
        override val headingVelRadPerSec: Float get() = dev.hVelocity
        val deviceStatus: Short get() = dev.deviceStatus
        val disconnected: Boolean get() = dev.disconnected

        override fun resetImu() = send(SRSHub.GoBildaPinpoint.ResetIMUCommand(bus))
        override fun setPose(xMm: Float, yMm: Float, headingRad: Float) =
            send(SRSHub.GoBildaPinpoint.SetPositionCommand(bus, xMm, yMm, headingRad))
    }
}
