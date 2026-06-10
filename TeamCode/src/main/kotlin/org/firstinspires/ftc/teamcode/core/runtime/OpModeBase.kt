package org.firstinspires.ftc.teamcode.core.runtime

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.core.subsystems.localization.PinpointDirect
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag

/**
 * Base for every op-mode in this codebase. A concrete op-mode fills in four
 * lifecycle hooks:
 *
 *  - [configure]   — register subsystems on [robot] and wire trigger bindings
 *  - [onInitLoop]  — optional, runs repeatedly between init and start. Use it
 *                    for auton/alliance selection, vision warm-up, and
 *                    pre-start health display. Subsystem `periodic()` runs
 *                    each init tick; commands and hardware writes do not.
 *  - [onStart]     — optional, runs the instant the start button is pressed
 *  - [onLoop]      — runs every tick after subsystem reads and input polling
 *                    but before Ivy commands and hardware writes
 *
 * Telemetry is doubled up: the Driver Station's built-in [Telemetry] and
 * Panels's dashboard telemetry are both driven from the same [TelemetryBag]
 * so nothing has to be logged twice.
 */
abstract class OpModeBase : LinearOpMode() {

    lateinit var robot: Robot
        private set

    lateinit var driver: GamepadEx
        private set

    lateinit var operator: GamepadEx
        private set

    /** Combined FTC Driver Station + Panels telemetry. Use [telemetryBag] for structured lines. */
    lateinit var joinedTelemetry: Telemetry
        private set

    lateinit var telemetryBag: TelemetryBag
        private set

    private val logTag = "OpModeBase"

    /** Override to pick a side for auton; defaults to RED (fine for teleop). */
    open val alliance: Alliance get() = Alliance.RED

    /** Hardware devices this op-mode expects before [configure] resolves them. */
    protected open val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard

    /** Register subsystems on [robot] and set up default commands. */
    protected abstract fun configure()

    /**
     * Called repeatedly while the op-mode sits in init (after [configure] /
     * [Robot.init], before start). Gamepad edges work ([driver] / [operator]
     * are updated each init tick), trigger bindings do not fire, and nothing
     * is written to hardware. Buffer telemetry via [telemetryBag]; it is
     * flushed automatically.
     */
    protected open fun onInitLoop() {}

    /** Called once on the first tick after start. */
    protected open fun onStart() {}

    /** Called every tick during the main loop. */
    protected open fun onLoop() {}

    /**
     * Set to false to suppress the auto-published "Loop" telemetry section.
     * Defaults on — the data is already collected by [Robot.profile] each
     * tick, so surfacing it costs only the telemetry put calls (which are
     * throttled by [TelemetryBag]).
     */
    protected open val publishLoopTelemetry: Boolean get() = true

    /** Set false for op-modes that want to own all health telemetry themselves. */
    protected open val publishHealthTelemetry: Boolean get() = true

    private var voltageSensor: VoltageSensor? = null
    private var cachedVoltage = Double.NaN
    private var voltageReadHealthTick = Long.MIN_VALUE
    private var healthTick = 0L

    private fun publishLoopProfile() {
        if (!publishLoopTelemetry) return
        val p = robot.profile
        telemetryBag.section("Loop") {
            // total/hz lag one tick: this runs inside the telemetry phase,
            // before the current tick's total is computed.
            put("hz", robot.loopHz, decimals = 1)
            put("count", robot.loopCount)
            put("total ms", p.totalNanos / 1e6, decimals = 2)
            put("total max ms", p.maxTotalNanos / 1e6, decimals = 2)
            put("clearCaches ms", p.clearCachesNanos / 1e6, decimals = 2)
            put("clearCaches max ms", p.maxClearCachesNanos / 1e6, decimals = 2)
            put("periodic ms", p.periodicNanos / 1e6, decimals = 2)
            put("periodic max ms", p.maxPeriodicNanos / 1e6, decimals = 2)
            put("input ms", p.inputNanos / 1e6, decimals = 2)
            put("input max ms", p.maxInputNanos / 1e6, decimals = 2)
            put("control ms", p.controlNanos / 1e6, decimals = 2)
            put("control max ms", p.maxControlNanos / 1e6, decimals = 2)
            put("scheduler ms", p.schedulerNanos / 1e6, decimals = 2)
            put("scheduler max ms", p.maxSchedulerNanos / 1e6, decimals = 2)
            put("writeHardware ms", p.writeHardwareNanos / 1e6, decimals = 2)
            put("writeHardware max ms", p.maxWriteHardwareNanos / 1e6, decimals = 2)
            put("telemetry ms", p.telemetryNanos / 1e6, decimals = 2)
            put("telemetry max ms", p.maxTelemetryNanos / 1e6, decimals = 2)
            put("overhead ms", p.overheadNanos / 1e6, decimals = 2)
            put("overhead max ms", p.maxOverheadNanos / 1e6, decimals = 2)
        }
    }

    private var telemetryFailures = 0

    /**
     * Telemetry must never be able to stop the robot: a Panels websocket
     * hiccup or a bad format string in a put gets logged and swallowed
     * instead of killing the op-mode mid-match.
     */
    private fun safeFlush(): Boolean = try {
        telemetryBag.flush()
    } catch (t: Throwable) {
        telemetryFailures++
        if (telemetryFailures <= 5) {
            RobotLog.ee(logTag, t, "Telemetry flush failed ($telemetryFailures)")
        }
        false
    }

    private fun reportLoopCrash(t: Throwable) {
        try {
            val message = "LOOP CRASHED: ${t.javaClass.simpleName}: ${t.message}"
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

    private fun cachedBatteryVoltage(): Double? {
        val sensor = voltageSensor ?: return null
        if (cachedVoltage.isNaN() || healthTick - voltageReadHealthTick >= 25) {
            cachedVoltage = sensor.voltage
            voltageReadHealthTick = healthTick
        }
        return cachedVoltage
    }

    private fun publishHealth(includeInitOnly: Boolean) {
        if (!publishHealthTelemetry) return
        healthTick++
        telemetryBag.section("Health") {
            val voltage = cachedBatteryVoltage()
            if (voltage != null) put("battery V", voltage, decimals = 2)
            for (subsystem in robot.subsystems()) {
                val health = subsystem.health()
                if (health != null) put(subsystem.name, health)
            }
            if (includeInitOnly) {
                val status = try {
                    PinpointDirect.status(hardwareMap).toString()
                } catch (t: Throwable) {
                    "${t.javaClass.simpleName}: ${t.message}"
                }
                put("Pinpoint status", status)
            }
        }
    }

    final override fun runOpMode() {
        robot = Robot(hardwareMap).also { it.alliance = alliance }
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        val panels = PanelsTelemetry.telemetry
        joinedTelemetry = JoinedTelemetry(telemetry, panels.wrapper)
        // The bag fans out to DS + Panels itself — handing it joinedTelemetry
        // (which also forwards to Panels via the wrapper) would double-log
        // every line on the dashboard.
        telemetryBag = TelemetryBag(telemetry, panels)

        try {
            Preflight.check(hardwareMap, requiredDevices)
            configure()
            robot.init()
            voltageSensor = firstVoltageSensor()

            telemetry.addLine("Init complete — ${robot.subsystems().size} subsystems")
            telemetry.update()

            while (opModeInInit()) {
                robot.initTick()
                // Edges work for selectors; trigger bindings must not fire
                // before start (Scheduler.schedule starts commands immediately).
                driver.update(pollTriggers = false)
                operator.update(pollTriggers = false)
                onInitLoop()
                publishHealth(includeInitOnly = true)
                safeFlush()
                sleep(20)
            }
        } catch (t: Throwable) {
            telemetry.addLine("INIT FAILED: ${t.javaClass.simpleName}: ${t.message}")
            telemetry.update()
            robot.stop()
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
            onStart()
            while (opModeIsActive()) {
                robot.loop(
                    input = {
                        driver.update()
                        operator.update()
                    },
                    control = { onLoop() },
                    telemetry = {
                        publishLoopProfile()
                        publishHealth(includeInitOnly = false)
                        if (safeFlush()) robot.profile.resetMaxima()
                    },
                )
            }
        } catch (t: Throwable) {
            reportLoopCrash(t)
            throw t
        } finally {
            robot.stop()
        }
    }
}
