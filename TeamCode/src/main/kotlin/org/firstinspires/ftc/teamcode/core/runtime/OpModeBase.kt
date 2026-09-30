package org.firstinspires.ftc.teamcode.core.runtime

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.input.GamepadEx
import org.firstinspires.ftc.teamcode.core.logging.FieldView
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveTelemetrySource
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerConfig

/**
 * Base for every op-mode in this codebase. A concrete op-mode fills in:
 *
 *  - [configure]   — register subsystems on [robot], set default commands,
 *                    and bind gamepad triggers to Ivy commands
 *  - [onInitLoop]  — optional, runs repeatedly between init and start.
 *                    Subsystem reads run; commands and hardware writes do not.
 *  - [onStart]     — optional, runs the instant start is pressed
 *  - [onLoop]      — every tick after reads and input, before commands and writes
 *
 * Telemetry goes to the Driver Station and Panels through one [TelemetryBag].
 */
abstract class OpModeBase : LinearOpMode() {

    lateinit var robot: Robot
        private set

    lateinit var driver: GamepadEx
        private set

    lateinit var operator: GamepadEx
        private set

    lateinit var telemetryBag: TelemetryBag
        private set

    private val logTag = "OpModeBase"

    /** Override to pick the side this op-mode runs on. */
    protected open val initialAlliance: Alliance get() = Alliance.RED

    /** Runtime alliance source of truth. */
    val alliance: Alliance get() = if (::robot.isInitialized) robot.alliance else initialAlliance

    /** Hardware devices this op-mode expects before [configure] resolves them. */
    protected open val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard

    /** Register subsystems on [robot], set default commands, and bind triggers. */
    protected abstract fun configure()

    /** Called repeatedly while the op-mode sits in init. Gamepad edges work; bindings do not fire. */
    protected open fun onInitLoop() {}

    /** Called once on the first tick after start. */
    protected open fun onStart() {}

    /** Called every tick during the main loop. */
    protected open fun onLoop() {}

    /** Set false to suppress the auto-published "Loop" telemetry section. */
    protected open val publishLoopTelemetry: Boolean get() = true

    /** Set false for op-modes that want to own all health telemetry themselves. */
    protected open val publishHealthTelemetry: Boolean get() = true

    /** Set false to suppress the live robot/path drawing on the Panels field view. */
    protected open val publishFieldView: Boolean get() = true

    /** Rumble both gamepads once when the match reaches endgame. */
    protected open val endgameRumble: Boolean get() = true

    private var voltageSensor: VoltageSensor? = null
    private var cachedVoltage = Double.NaN
    private var lastVoltageReadNs = Long.MIN_VALUE
    private var lastConfigPersistNs = Long.MIN_VALUE
    private val fieldView = FieldView()
    private var fieldViewDrive: DriveTelemetrySource? = null
    private var startNanos = 0L
    private var endgameRumbled = false
    private var telemetryFailures = 0

    private fun publishLoopProfile() {
        if (!publishLoopTelemetry) return
        val p = robot.profile
        telemetryBag.section("Loop") {
            // total/hz lag one tick: this runs before the tick's total is known.
            put("hz", robot.loopHz, decimals = 1)
            put("count", robot.loopCount)
            put("total ms", p.totalNanos / 1e6, decimals = 2)
            put("total max ms", p.maxTotalNanos / 1e6, decimals = 2)
            for (phase in LoopPhase.entries) {
                put("${phase.label} ms", p[phase] / 1e6, decimals = 2)
                put("${phase.label} max ms", p.max(phase) / 1e6, decimals = 2)
            }
            put("overhead ms", p.overheadNanos / 1e6, decimals = 2)
            put("overhead max ms", p.maxOverheadNanos / 1e6, decimals = 2)
        }
    }

    /** Telemetry must never stop the robot: a Panels hiccup is logged and swallowed. */
    private fun safeFlush(): Boolean = try {
        telemetryBag.flush()
    } catch (t: Throwable) {
        telemetryFailures++
        if (telemetryFailures <= 5) RobotLog.ee(logTag, t, "Telemetry flush failed ($telemetryFailures)")
        false
    }

    private inline fun safeTelemetry(block: () -> Unit) {
        try {
            block()
        } catch (t: Throwable) {
            telemetryFailures++
            if (telemetryFailures <= 5) RobotLog.ee(logTag, t, "Init telemetry failed ($telemetryFailures)")
        }
    }

    /** Hardware is already stopped when this runs; the trace is already in the WPILOG `events` channel. */
    private fun reportLoopCrash(t: Throwable, message: String) {
        try {
            telemetry.addLine(message)
            telemetry.update()
            RobotLog.ee(logTag, t, message)
        } catch (_: Throwable) {
            // Preserve the original loop exception even if diagnostics fail.
        }
    }

    private fun firstVoltageSensor(): VoltageSensor? = try {
        val iterator = hardwareMap.voltageSensor.iterator()
        if (iterator.hasNext()) iterator.next() else null
    } catch (_: Throwable) {
        null
    }

    /** Battery voltage, throttled to one read per 250 ms; runs regardless of telemetry flags. */
    private fun refreshVoltage() {
        val sensor = voltageSensor ?: return
        val now = System.nanoTime()
        if (!cachedVoltage.isNaN() && lastVoltageReadNs != Long.MIN_VALUE && now - lastVoltageReadNs < 250_000_000L) {
            return
        }
        cachedVoltage = sensor.voltage
        lastVoltageReadNs = now
    }

    private fun publishHealth(includeInitOnly: Boolean) {
        if (!publishHealthTelemetry) return
        telemetryBag.section("Health") {
            if (!cachedVoltage.isNaN()) put("battery V", cachedVoltage, decimals = 2)
            if (robot.commandFaultCount > 0) {
                val last = robot.lastCommandFault
                put("command faults", "${robot.commandFaultCount} (last: ${last?.javaClass?.simpleName}: ${last?.message})")
            }
            for (subsystem in robot.subsystems()) {
                val health = subsystem.health()
                if (health != null) put(subsystem.name, health)
            }
            if (includeInitOnly && !cachedVoltage.isNaN() && cachedVoltage < LOW_BATTERY_WARN_VOLTS) {
                put("battery WARNING", "LOW — swap before the match")
            }
        }
    }

    private fun updateEndgameRumble() {
        if (!endgameRumble || endgameRumbled) return
        if (robot.clock.nanos() - startNanos >= ENDGAME_START_NANOS) {
            driver.rumbleBlips(3)
            operator.rumbleBlips(3)
            endgameRumbled = true
        }
    }

    final override fun runOpMode() {
        robot = Robot(hardwareMap).also { it.alliance = initialAlliance }
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        // Restore live-tuned values before configure() reads any of them.
        ConfigStore.register("drive", DriveConfig, DriveConfig::resetDefaults)
        ConfigStore.register("localizer", LocalizerConfig, LocalizerConfig::resetDefaults)
        ConfigStore.loadFromDisk()

        val panels = PanelsTelemetry.telemetry
        telemetryBag = TelemetryBag(telemetry, panels)
        robot.enableFlightRecorder(
            javaClass.simpleName,
            driver = { driver },
            operator = { operator },
            batteryVoltage = { if (cachedVoltage.isNaN()) null else cachedVoltage },
        )

        try {
            Preflight.check(hardwareMap, requiredDevices)
            configure()
            // configure() may have registered season config objects; restore them too.
            ConfigStore.loadFromDisk()
            robot.init()
            voltageSensor = firstVoltageSensor()
            fieldViewDrive = robot.subsystems().firstOrNull { it is DriveTelemetrySource } as? DriveTelemetrySource

            telemetry.addLine("Init complete — ${robot.subsystems().size} subsystems")
            telemetry.update()

            while (opModeInInit()) {
                robot.initTick()
                // Edges work for selectors; bindings must not start commands before start.
                driver.update(pollTriggers = false)
                operator.update(pollTriggers = false)
                onInitLoop()
                safeTelemetry {
                    refreshVoltage()
                    publishHealth(includeInitOnly = true)
                    if (publishFieldView) fieldView.draw(fieldViewDrive)
                }
                safeFlush()
                sleep(20)
            }
        } catch (t: Throwable) {
            robot.stop {
                telemetry.addLine("INIT FAILED: ${t.javaClass.simpleName}: ${t.message}")
                telemetry.update()
            }
            throw t
        }

        if (isStopRequested) {
            robot.stop()
            return
        }

        driver.lockBindings()
        operator.lockBindings()

        try {
            robot.start()
            startNanos = robot.clock.nanos()
            endgameRumbled = false
            robot.recordEvent("start")
            onStart()
            while (opModeIsActive()) {
                robot.loop(
                    input = {
                        driver.update()
                        operator.update()
                    },
                    control = {
                        onLoop()
                        updateEndgameRumble()
                    },
                    telemetry = {
                        refreshVoltage()
                        publishLoopProfile()
                        publishHealth(includeInitOnly = false)
                        if (publishFieldView) fieldView.draw(fieldViewDrive)
                        if (safeFlush()) robot.profile.resetMaxima()
                        persistConfigThrottled()
                    },
                )
            }
        } catch (t: Throwable) {
            // Exceptions and Errors alike (TODO() is a NotImplementedError).
            robot.stopAfterCrash(t) { reportLoopCrash(t, it) }
            throw t
        } finally {
            robot.stop()
        }
    }

    /** Persist Panels-tuned config values at most once per second. */
    private fun persistConfigThrottled() {
        val now = System.nanoTime()
        if (lastConfigPersistNs != Long.MIN_VALUE && now - lastConfigPersistNs < CONFIG_PERSIST_INTERVAL_NS) return
        lastConfigPersistNs = now
        try {
            ConfigStore.persistIfDirty()
        } catch (t: Throwable) {
            RobotLog.ee(logTag, t, "Config persist failed")
        }
    }

    private companion object {
        /** Resting voltage below which the init screen warns to swap the battery. */
        const val LOW_BATTERY_WARN_VOLTS = 12.0
        const val CONFIG_PERSIST_INTERVAL_NS = 1_000_000_000L
        /** Endgame is the last 30 s of the 2:00 driver period. */
        const val ENDGAME_START_NANOS = 90_000_000_000L
    }
}

/**
 * Validates that every device the op-mode needs exists in the active
 * configuration *before* anything tries to resolve it. The worst bring-up
 * experience is a wiring typo surfacing as an opaque crash inside Pedro's
 * FollowerBuilder; this converts it into one message listing every missing
 * device at once.
 *
 * [OpModeBase] runs the check before `configure()`. Op-modes that need more
 * devices override `requiredDevices` and append their own [Requirement]s.
 */
object Preflight {

    data class Requirement(val name: String, val type: Class<out HardwareDevice>)

    /** The devices every op-mode in this template needs: drive + localizer. */
    val standard: List<Requirement> = listOf(
        Requirement(RobotConfig.Drive.FRONT_LEFT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.FRONT_RIGHT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.BACK_LEFT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.BACK_RIGHT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Localization.PINPOINT, GoBildaPinpointDriver::class.java),
    )

    /** Throws [HardwareConfigError] listing every missing requirement. */
    fun check(hardwareMap: HardwareMap, requirements: List<Requirement>) {
        val missing = requirements.filter { hardwareMap.tryGet(it.type, it.name) == null }
        if (missing.isEmpty()) return
        val listing = missing.joinToString("\n") { "  - ${it.type.simpleName} \"${it.name}\"" }
        throw HardwareConfigError(
            "Active configuration is missing ${missing.size} device(s):\n$listing",
        )
    }
}

class HardwareConfigError(message: String, cause: Throwable? = null) : RuntimeException(message, cause)
