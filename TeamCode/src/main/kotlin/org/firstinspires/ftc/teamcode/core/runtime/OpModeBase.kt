package org.firstinspires.ftc.teamcode.core.runtime

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.RobotLog
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.core.logging.FieldView
import org.firstinspires.ftc.teamcode.core.logging.SchedulerIntrospection
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.PinpointDirect
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.MatchTimer
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

    /** Override to pick the default side before an init-loop selector changes it. */
    protected open val initialAlliance: Alliance get() = Alliance.RED

    /** Runtime alliance source of truth. Init-loop selectors update [robot]. */
    val alliance: Alliance get() = if (::robot.isInitialized) robot.alliance else initialAlliance

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

    /** Set false to suppress the live robot/path drawing on the Panels field view. */
    protected open val publishFieldView: Boolean get() = true

    /** Rumble both gamepads once when the match reaches endgame. */
    protected open val endgameRumble: Boolean get() = true

    /**
     * When true, command-layer exceptions are contained instead of killing
     * the op-mode — see [Robot.containCommandFaults]. [org.firstinspires.ftc.teamcode.opmodes.TeleOpBase]
     * turns this on; auton op-modes keep the fail-fast default.
     */
    protected open val containCommandFaults: Boolean get() = false

    private var voltageSensor: VoltageSensor? = null
    private var cachedVoltage = Double.NaN
    private var lastVoltageReadNs = Long.MIN_VALUE
    private val fieldView = FieldView()
    private var fieldViewDrive: MecanumDriveSubsystem? = null
    private val logDir = File("/sdcard/FIRST/logs")
    private val lastCrashFile = File(logDir, "lastcrash.txt")
    protected val matchTimer = MatchTimer()
    private var endgameRumbled = false

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
            put("record ms", p.recordNanos / 1e6, decimals = 2)
            put("record max ms", p.maxRecordNanos / 1e6, decimals = 2)
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
            val trace = StringWriter().also { writer ->
                t.printStackTrace(PrintWriter(writer))
            }.toString()
            // "What was happening" beats "where it died": capture scheduler
            // state and the recent event timeline before anything shuts down.
            val running = try {
                SchedulerIntrospection.DEFAULT.runningCommandNames()
            } catch (_: Throwable) {
                emptyList()
            }
            val report = buildString {
                appendLine(message)
                appendLine(
                    "loop ${robot.loopCount}, ${"%.1f".format(Locale.US, matchTimer.elapsedSec)} s into the match, " +
                        "${robot.commandFaultCount} contained fault(s)",
                )
                if (running.isNotEmpty()) {
                    appendLine("running commands:")
                    for (name in running) appendLine("  $name")
                }
                val events = robot.recentEvents()
                if (events.isNotEmpty()) {
                    appendLine("recent events:")
                    for (event in events) appendLine("  $event")
                }
                append(trace)
            }
            robot.recordEvent(report)
            robot.closeFlightRecorder()
            writeLastCrash(report)
            telemetry.addLine(message)
            telemetry.update()
            RobotLog.ee(logTag, t, message)
        } catch (_: Throwable) {
            // Preserve the original loop exception even if diagnostics fail.
        }
    }

    private fun surfacePreviousCrash() {
        try {
            if (!lastCrashFile.exists()) return
            val firstLine = lastCrashFile.useLines { lines -> lines.firstOrNull() }
            if (!firstLine.isNullOrBlank()) {
                telemetry.addLine("previous run crashed: $firstLine")
                telemetry.update()
            }
            lastCrashFile.delete()
        } catch (t: Throwable) {
            RobotLog.ee(logTag, t, "Failed to surface previous crash")
        }
    }

    private fun writeLastCrash(contents: String) {
        try {
            logDir.mkdirs()
            lastCrashFile.writeText(contents)
        } catch (t: Throwable) {
            RobotLog.ee(logTag, t, "Failed to write lastcrash.txt")
        }
    }

    private fun firstVoltageSensor(): VoltageSensor? = try {
        val iterator = hardwareMap.voltageSensor.iterator()
        if (iterator.hasNext()) iterator.next() else null
    } catch (_: Throwable) {
        null
    }

    /**
     * Refresh the cached battery voltage, throttled to one hardware read per
     * 250 ms. Runs every tick regardless of [publishHealthTelemetry] so the
     * flight recorder's battery channel never silently depends on a
     * telemetry flag.
     */
    private fun refreshVoltage() {
        val sensor = voltageSensor ?: return
        val now = System.nanoTime()
        if (!cachedVoltage.isNaN() &&
            lastVoltageReadNs != Long.MIN_VALUE &&
            now - lastVoltageReadNs < 250_000_000L
        ) {
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
                put(
                    "command faults",
                    "${robot.commandFaultCount} (last: ${last?.javaClass?.simpleName}: ${last?.message})",
                )
            }
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
                if (!cachedVoltage.isNaN() && cachedVoltage < LOW_BATTERY_WARN_VOLTS) {
                    put("battery WARNING", "LOW — swap before the match")
                }
            }
        }
    }

    private fun updateEndgameRumble() {
        if (!endgameRumble || endgameRumbled) return
        if (matchTimer.inEndgame()) {
            driver.rumbleBlips(3)
            operator.rumbleBlips(3)
            endgameRumbled = true
        }
    }

    final override fun runOpMode() {
        robot = Robot(hardwareMap).also {
            it.alliance = initialAlliance
            it.containCommandFaults = containCommandFaults
        }
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        val panels = PanelsTelemetry.telemetry
        joinedTelemetry = JoinedTelemetry(telemetry, panels.wrapper)
        // The bag fans out to DS + Panels itself — handing it joinedTelemetry
        // (which also forwards to Panels via the wrapper) would double-log
        // every line on the dashboard.
        telemetryBag = TelemetryBag(telemetry, panels)
        robot.enableFlightRecorder(
            javaClass.simpleName,
            driver = { driver },
            operator = { operator },
            batteryVoltage = { if (cachedVoltage.isNaN()) null else cachedVoltage },
        )
        surfacePreviousCrash()

        try {
            Preflight.check(hardwareMap, requiredDevices)
            configure()
            robot.init()
            voltageSensor = firstVoltageSensor()
            fieldViewDrive =
                robot.subsystems().firstOrNull { it is MecanumDriveSubsystem } as? MecanumDriveSubsystem

            telemetry.addLine("Init complete — ${robot.subsystems().size} subsystems")
            telemetry.update()

            while (opModeInInit()) {
                robot.initTick()
                // Edges work for selectors; trigger bindings must not fire
                // before start (Scheduler.schedule starts commands immediately).
                driver.update(pollTriggers = false)
                operator.update(pollTriggers = false)
                onInitLoop()
                refreshVoltage()
                publishHealth(includeInitOnly = true)
                if (publishFieldView) fieldView.draw(fieldViewDrive)
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
            matchTimer.start()
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

    private companion object {
        /** Resting voltage below which the init screen warns to swap the battery. */
        const val LOW_BATTERY_WARN_VOLTS = 12.0
    }
}
