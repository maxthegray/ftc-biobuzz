package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.io.MotorIO
import org.firstinspires.ftc.teamcode.core.io.RealMotorIO
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.GamepadEx

/**
 * Live-tuning sandbox: spins one motor at a power typed into Panels.
 *
 * Hold the right bumper to run; releasing it stops the motor. The power
 * itself is a [MotorTestConfig] field, so it is persisted by
 * [ConfigStore] and survives power cycles and Sloth hot reloads — which
 * also means it is whatever you left it at last session, hence the
 * dead-man button.
 *
 * Telemetry pairs commanded power with measured velocity, so this doubles
 * as the rig for a feedforward ramp: hold a power, let velocity settle,
 * record the pair, repeat. The slope of power against steady-state
 * velocity is kV; the intercept is kS (+ kG on a vertical mechanism).
 */
@TeleOp(name = "Starter: Panels Motor Spin", group = "Starter Diagnostics")
class PanelsMotorSpinTeleOp : OpModeBase() {

    private lateinit var spin: MotorSpinSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(MOTOR_NAME, DcMotorEx::class.java))
    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        ConfigStore.register("motorTest", MotorTestConfig)
        spin = robot.register(MotorSpinSubsystem(MOTOR_NAME))
        driver.button(GamepadEx.Button.RIGHT_BUMPER).whileTrue(spin.spinCommand())
    }

    override fun onInitLoop() = emitTelemetry()

    override fun onLoop() = emitTelemetry()

    private fun emitTelemetry() {
        telemetryBag.section("Panels Motor Spin") {
            put("SAFETY", "ROBOT ON BLOCKS")
            put("motor", MOTOR_NAME)
            put("controls", "hold RIGHT BUMPER to spin")
            put("running", spin.running)
            put("panels power", MotorTestConfig.power, decimals = 3)
            put("max power", MotorTestConfig.safeMaxPower, decimals = 3)
            put("applied power", spin.appliedPower, decimals = 3)
        }
        telemetryBag.section("Measured") {
            put("velocity ticks/s", spin.velocityTicksPerSec, decimals = 1)
            put("position ticks", spin.positionTicks, decimals = 0)
        }
    }

    private companion object {
        /** Change this to whichever motor you want to spin. */
        const val MOTOR_NAME = RobotConfig.Drive.FRONT_LEFT_MOTOR
    }
}

/**
 * Panels-tunable knobs for [PanelsMotorSpinTeleOp], persisted under the
 * `motorTest` section.
 */
@Configurable
object MotorTestConfig {

    private const val DEFAULT_POWER = 0.0
    private const val DEFAULT_MAX_POWER = 0.3

    /** Power applied while the dead-man button is held. Clamped to ±[maxPower]. */
    @JvmField var power: Double = DEFAULT_POWER

    /**
     * Ceiling on the magnitude of [power]. Typing 1.0 into Panels when you
     * meant 0.1 should not launch the mechanism across the shop.
     */
    @JvmField var maxPower: Double = DEFAULT_MAX_POWER

    internal val safeMaxPower: Double
        get() = if (maxPower.isFinite()) maxPower.coerceIn(0.0, 1.0) else DEFAULT_MAX_POWER

    internal val safePower: Double
        get() = if (power.isFinite()) power.coerceIn(-safeMaxPower, safeMaxPower) else 0.0
}

private class MotorSpinSubsystem(private val motorName: String) : SubsystemBase("MotorSpin") {

    private lateinit var io: MotorIO

    var appliedPower: Double = 0.0
        private set
    var running: Boolean = false
        private set
    var velocityTicksPerSec: Double = 0.0
        private set
    var positionTicks: Double = 0.0
        private set

    override fun init(hardwareMap: HardwareMap) {
        io = RealMotorIO(DeviceReaders.motor(hardwareMap, motorName))
    }

    override fun periodic() {
        velocityTicksPerSec = io.velocityTicksPerSec
        positionTicks = io.positionTicks
    }

    fun spinCommand(): Command = Command.build()
        .setName("panels motor spin")
        .requiring(this)
        .setStart { running = true }
        .setExecute { appliedPower = MotorTestConfig.safePower }
        .setDone { false }
        .setEnd {
            appliedPower = 0.0
            running = false
        }

    override fun writeHardware() {
        io.setPower(appliedPower)
    }

    override fun health(): String =
        "$motorName power=${"%.2f".format(Locale.US, appliedPower)}"

    override fun logState(log: StateLog) {
        log.put("motor", motorName)
        log.put("running", running)
        log.put("panelsPower", MotorTestConfig.power)
        log.put("appliedPower", appliedPower)
        log.put("velocityTicksPerSec", velocityTicksPerSec)
        log.put("positionTicks", positionTicks)
    }

    override fun stop() {
        appliedPower = 0.0
        running = false
        if (!::io.isInitialized) return
        try {
            io.setPower(0.0)
        } catch (_: Throwable) {
            // Best-effort stop; the op-mode is ending regardless.
        }
    }
}
