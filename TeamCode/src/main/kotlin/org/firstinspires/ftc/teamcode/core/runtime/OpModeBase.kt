package org.firstinspires.ftc.teamcode.core.runtime

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.GamepadEx
import org.firstinspires.ftc.teamcode.core.util.TelemetryBag

/**
 * Base for every op-mode in this codebase. A concrete op-mode fills in three
 * lifecycle hooks:
 *
 *  - [configure] — register subsystems on [robot] and wire default commands
 *  - [onStart]   — optional, runs the instant the start button is pressed
 *  - [onLoop]    — runs every tick after subsystem reads but before Ivy
 *                  commands and hardware writes. Use it for gamepad handling,
 *                  scheduling commands, and telemetry.
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

    /** Override to pick a side for auton; defaults to RED (fine for teleop). */
    open val alliance: Alliance get() = Alliance.RED

    /** Register subsystems on [robot] and set up default commands. */
    protected abstract fun configure()

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

    private fun publishLoopProfile() {
        if (!publishLoopTelemetry) return
        val p = robot.profile
        telemetryBag.section("Loop") {
            put("hz", robot.loopHz, decimals = 1)
            put("total ms", p.totalNanos / 1e6, decimals = 2)
            put("clearCaches ms", p.clearCachesNanos / 1e6, decimals = 2)
            put("periodic ms", p.periodicNanos / 1e6, decimals = 2)
            put("control ms", p.controlNanos / 1e6, decimals = 2)
            put("scheduler ms", p.schedulerNanos / 1e6, decimals = 2)
            put("writeHardware ms", p.writeHardwareNanos / 1e6, decimals = 2)
            put("overhead ms", p.overheadNanos / 1e6, decimals = 2)
        }
    }

    final override fun runOpMode() {
        robot = Robot(hardwareMap).also { it.alliance = alliance }
        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        val panels = PanelsTelemetry.telemetry
        joinedTelemetry = JoinedTelemetry(telemetry, panels.wrapper)
        telemetryBag = TelemetryBag(joinedTelemetry, panels)

        try {
            configure()
            robot.init()
        } catch (t: Throwable) {
            telemetry.addLine("INIT FAILED: ${t.javaClass.simpleName}: ${t.message}")
            telemetry.update()
            robot.stop()
            throw t
        }

        telemetry.addLine("Init complete — ${robot.subsystems().size} subsystems")
        telemetry.update()

        waitForStart()
        if (isStopRequested) {
            robot.stop()
            return
        }

        try {
            robot.start()
            onStart()
            while (opModeIsActive()) {
                driver.update()
                operator.update()
                robot.loop { onLoop() }
                publishLoopProfile()
                telemetryBag.flush()
            }
        } finally {
            robot.stop()
        }
    }
}
