package org.firstinspires.ftc.teamcode.core

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import kotlin.math.cos
import kotlin.math.sin
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.subsystems.DriveTelemetrySource

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

    /** Set false to suppress the live robot/path drawing on the Panels field view. */
    protected open val publishFieldView: Boolean get() = true

    private var cachedVoltage = Double.NaN
    private val fieldView = FieldView()
    private var fieldViewDrive: DriveTelemetrySource? = null
    private var telemetryFailures = 0

    /** The phase breakdown is in the flight log; `tools/logs/analyze_wpilog.py` reports it. */
    private fun publishLoopTiming() {
        val p = robot.profile
        telemetryBag.section("Loop") {
            put("hz", robot.loopHz, decimals = 1)
            put("max ms", p.maxTotalNanos / 1e6, decimals = 1)
        }
    }

    /** Telemetry must never stop the robot: a Panels hiccup is logged and swallowed. */
    private inline fun safeTelemetry(block: () -> Unit) {
        try {
            block()
        } catch (t: Throwable) {
            telemetryFailures++
            if (telemetryFailures <= 5) RobotLog.ee(logTag, t, "Telemetry failed ($telemetryFailures)")
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

    /** Battery voltage: the Control Hub's input voltage, which [Robot.hubPower] reads anyway. */
    private fun refreshVoltage() {
        val hubVolts = robot.hubPower.batteryVolts
        if (hubVolts.isFinite()) cachedVoltage = hubVolts
    }

    private fun publishHealth(includeInitOnly: Boolean) {
        telemetryBag.section("Health") {
            if (!cachedVoltage.isNaN()) put("battery V", cachedVoltage, decimals = 2)
            if (robot.commandFaultCount > 0) {
                val last = robot.lastCommandFault
                put("command faults", "${robot.commandFaultCount} (last: ${last?.javaClass?.simpleName}: ${last?.message})")
            }
            if (robot.subsystemFaultCount > 0) {
                val last = robot.lastSubsystemFault
                put("subsystem faults", "${robot.subsystemFaultCount} (last: ${last?.javaClass?.simpleName}: ${last?.message})")
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

    final override fun runOpMode() {
        robot = Robot(hardwareMap).also { it.alliance = initialAlliance }
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

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
            robot.init()
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
                    telemetryBag.flush()
                }
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
            robot.recordEvent("start")
            onStart()
            while (opModeIsActive()) {
                robot.loop(
                    input = {
                        driver.update()
                        operator.update()
                    },
                    control = { onLoop() },
                    telemetry = {
                        refreshVoltage()
                        publishLoopTiming()
                        publishHealth(includeInitOnly = false)
                        if (publishFieldView) fieldView.draw(fieldViewDrive)
                        if (telemetryBag.flush()) robot.profile.resetMaxima()
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

    private companion object {
        /** Resting voltage below which the init screen warns to swap the battery. */
        const val LOW_BATTERY_WARN_VOLTS = 12.0
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

/**
 * Live field view on the Panels dashboard for *real* op-modes (the upstream
 * Pedro `Tuning` op-mode draws its own): robot pose + heading every redraw,
 * plus the active path while the drive is following. This is the primary
 * "where does the robot think it is" debugging surface during auton
 * development and matches. Consumes [DriveTelemetrySource], so it never
 * touches Pedro types directly.
 *
 * Redraws are throttled to 10 Hz — pushing the field websocket
 * at loop rate is pure overhead. Drawing must never stop the robot: any
 * failure is counted, and the view disables itself after a few, mirroring
 * the telemetry-flush policy in [OpModeBase].
 */
private class FieldView {
    private var lastDrawNs = Long.MIN_VALUE
    private var failures = 0

    private var field: FieldManager? = null

    private val robotStyle = Style("", "#3F51B5", 0.75)
    private val pathStyle = Style("", "#4CAF50", 0.75)

    fun draw(drive: DriveTelemetrySource?) {
        if (drive == null || failures >= MAX_FAILURES) return
        val now = System.nanoTime()
        if (lastDrawNs != Long.MIN_VALUE && now - lastDrawNs < REDRAW_INTERVAL_NS) return
        lastDrawNs = now

        try {
            val manager = field ?: PanelsField.field.also {
                it.setOffsets(PanelsField.presets.PEDRO_PATHING)
                field = it
            }
            drawPath(manager, drive.currentPathPoints(PATH_SAMPLES))
            drawRobot(manager, drive.pose)
            manager.update()
        } catch (t: Throwable) {
            failures++
            if (failures == MAX_FAILURES) {
                RobotLog.ee("FieldView", t, "Field view disabled after $failures failures")
            }
        }
    }

    private fun drawPath(manager: FieldManager, points: List<Vector2D>) {
        // FieldManager.line() draws from the cursor but does not advance it,
        // so each segment re-anchors the cursor at the previous sample.
        manager.setStyle(pathStyle)
        var prev: Vector2D? = null
        for (point in points) {
            if (point.x().isNaN() || point.y().isNaN()) continue
            prev?.let {
                manager.moveCursor(it.x(), it.y())
                manager.line(point.x(), point.y())
            }
            prev = point
        }
    }

    private fun drawRobot(manager: FieldManager, pose: Pose) {
        val x = pose.x()
        val y = pose.y()
        val heading = pose.heading()
        if (x.isNaN() || y.isNaN() || heading.isNaN()) return

        manager.setStyle(robotStyle)
        manager.moveCursor(x, y)
        manager.circle(ROBOT_RADIUS)

        val hx = cos(heading) * ROBOT_RADIUS
        val hy = sin(heading) * ROBOT_RADIUS
        manager.setStyle(robotStyle)
        manager.moveCursor(x + hx / 2.0, y + hy / 2.0)
        manager.line(x + hx, y + hy)
    }

    private companion object {
        const val REDRAW_INTERVAL_NS = 100_000_000L
        const val ROBOT_RADIUS = 9.0
        const val MAX_FAILURES = 5
        const val PATH_SAMPLES = 16
    }
}
