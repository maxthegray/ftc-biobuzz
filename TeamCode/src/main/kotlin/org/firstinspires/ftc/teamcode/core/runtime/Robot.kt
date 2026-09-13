package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.core.logging.FlightRecorder
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.core.util.GamepadEx

/**
 * The hardware, subsystems and loop of one op-mode run.
 *
 *  - Owns the list of [SubsystemBase]s and their lifecycle
 *  - Puts every Lynx module in MANUAL bulk caching and clears the cache once per tick
 *  - Runs Ivy's static [Scheduler] at a fixed point in the loop, and resets it
 *    on construction and stop so no command outlives its op-mode
 *  - Owns the optional [FlightRecorder]
 *
 * [OpModeBase] drives this; you rarely construct Robot yourself.
 */
class Robot(
    val hardwareMap: HardwareMap,
    val clock: Clock = Clock.SYSTEM,
) {

    private val subsystems = mutableListOf<SubsystemBase>()
    val bulkRead = BulkReadManager(hardwareMap)

    var alliance: Alliance = Alliance.RED

    init {
        // Ivy's scheduler is a static singleton that survives op-modes, tests
        // and Sloth reloads. Nothing from a previous run may execute here.
        Scheduler.reset()
    }

    /** Commands aborted by a thrown exception this op-mode. */
    var commandFaultCount: Int = 0
        private set

    var lastCommandFault: Throwable? = null
        private set

    var telemetryFaultCount: Int = 0
        private set

    var recorderFaultCount: Int = 0
        private set

    /** Monotonic tick counter. */
    var loopCount: Long = 0
        private set

    /** Wall-clock duration of the most recent loop in nanoseconds. */
    var lastLoopNanos: Long = 0
        private set

    /** Per-phase breakdown of the most recent loop. */
    val profile = LoopProfile()

    private var lastTickEndNs: Long = 0
    private var flightRecorder: FlightRecorder? = null
    private var initialized = false
    private var stopped = false
    private var lastOverrunEventNs = Long.MIN_VALUE
    private var lastTelemetryFaultEventNs = Long.MIN_VALUE
    private var lastCommandFaultEventNs = Long.MIN_VALUE
    private var lastCommandFaultMessage: String? = null

    /**
     * Register a subsystem. Must happen before [init] (in the op-mode's
     * `configure()`); a later registration would never get its `init`.
     */
    fun <T : SubsystemBase> register(subsystem: T): T {
        check(!initialized) {
            "Cannot register ${subsystem.name} after Robot.init() — register subsystems in configure()."
        }
        subsystem.registerAfter?.let { required ->
            check(subsystems.any { required.isInstance(it) }) {
                "${subsystem.name} must be registered after a ${required.simpleName} — " +
                    "its writeHardware() depends on the ${required.simpleName} having run first this tick."
            }
        }
        subsystems += subsystem
        return subsystem
    }

    fun subsystems(): List<SubsystemBase> = subsystems

    fun enableFlightRecorder(
        opModeClassName: String,
        driver: () -> GamepadEx?,
        operator: () -> GamepadEx?,
        batteryVoltage: () -> Double?,
        directory: java.io.File = java.io.File("/sdcard/FIRST/logs"),
    ) {
        flightRecorder = FlightRecorder.open(
            opModeClassName,
            driver,
            operator,
            batteryVoltage,
            clock = clock,
            directory = directory,
        )
    }

    /** Write a timestamped message to the WPILOG `events` channel. */
    fun recordEvent(message: String) {
        flightRecorder?.event(message)
    }

    fun closeFlightRecorder() {
        flightRecorder?.close()
        flightRecorder = null
    }

    /** Initialise every registered subsystem. Exceptions propagate so init fails loudly. */
    fun init() {
        initialized = true
        bulkRead.init()
        for (s in subsystems) s.init(hardwareMap)
    }

    /** The op-mode started: reset the loop timer so the first tick isn't skewed by init time. */
    fun start() {
        lastTickEndNs = clock.nanos()
        loopCount = 0
    }

    /**
     * One main-loop tick. The order is fixed:
     *  1. Clear Lynx bulk caches so all reads this tick return fresh data
     *  2. [SubsystemBase.periodic] — reads and state updates
     *  3. [input] — gamepad edges and trigger bindings
     *  4. [control] — the op-mode's `onLoop()`
     *  5. Default commands, then [Scheduler.execute] — commands decide targets
     *  6. [SubsystemBase.writeHardware] — targets reach the hardware
     *  7. [telemetry] — observe-only; a failure is counted, never fatal
     *  8. Flight recorder sample
     *
     * An exception from phase 3 or 5 aborts every command: the scheduler is
     * cleared, every subsystem gets [SubsystemBase.onCommandFault], and the
     * reason is recorded. The loop continues and defaults resume next tick.
     * Exceptions from 2, 4 and 6 end the op-mode.
     */
    fun loop(
        input: () -> Unit = {},
        control: () -> Unit = {},
        telemetry: () -> Unit = {},
    ): Long {
        check(!stopped) { "Cannot loop a stopped Robot" }
        var phaseStart = clock.nanos()

        bulkRead.clearCaches()
        phaseStart = mark(phaseStart, LoopPhase.CLEAR_CACHES)

        for (s in subsystems) s.periodic()
        phaseStart = mark(phaseStart, LoopPhase.PERIODIC)

        commandPhase(input)
        phaseStart = mark(phaseStart, LoopPhase.INPUT)

        control()
        phaseStart = mark(phaseStart, LoopPhase.CONTROL)

        commandPhase {
            for (s in subsystems) {
                val default = s.defaultCommand ?: continue
                if (!Scheduler.isScheduled(default)) Scheduler.schedule(default)
            }
            Scheduler.execute()
        }
        phaseStart = mark(phaseStart, LoopPhase.SCHEDULER)

        for (s in subsystems) s.writeHardware()
        phaseStart = mark(phaseStart, LoopPhase.WRITE_HARDWARE)

        try {
            telemetry()
        } catch (t: Throwable) {
            recordTelemetryFault(t)
        }
        profile[LoopPhase.TELEMETRY] = clock.nanos() - phaseStart

        val recorder = flightRecorder
        if (recorder == null) {
            profile[LoopPhase.RECORD] = 0
        } else {
            // The recorder can't time its own final write, so the first
            // reading is logged and the second completes the profile.
            val recordStart = clock.nanos()
            try {
                recorder.record(this)
                recorder.recordRecorderNanos(clock.nanos() - recordStart)
            } catch (t: Throwable) {
                recordRecorderFault(t)
            }
            profile[LoopPhase.RECORD] = clock.nanos() - recordStart
        }
        val now = clock.nanos()

        lastLoopNanos = now - lastTickEndNs
        profile.totalNanos = lastLoopNanos
        lastTickEndNs = now
        loopCount++

        if (lastLoopNanos > LOOP_OVERRUN_NANOS && throttle(lastOverrunEventNs, now)) {
            lastOverrunEventNs = now
            recordEvent("LOOP OVERRUN: ${lastLoopNanos / 1_000_000} ms")
        }
        return lastLoopNanos
    }

    /**
     * One init-phase tick: fresh bulk read and subsystem reads. No commands
     * run and nothing is written, so the robot cannot move before start.
     */
    fun initTick() {
        bulkRead.clearCaches()
        for (s in subsystems) s.initPeriodic()
    }

    private fun mark(start: Long, phase: LoopPhase): Long {
        val now = clock.nanos()
        profile[phase] = now - start
        return now
    }

    private inline fun commandPhase(block: () -> Unit) {
        try {
            block()
        } catch (e: Exception) {
            abortCommands(e)
        }
    }

    /**
     * Ivy has no per-command fault isolation, and its state is not trustworthy
     * after a throw mid-execute. Clear everything, stop every mechanism, and
     * record why. No command end handler runs.
     */
    private fun abortCommands(e: Exception) {
        try { Scheduler.reset() } catch (_: Throwable) { /* keep safing */ }
        for (s in subsystems) {
            try { s.onCommandFault() } catch (_: Throwable) { /* safe every subsystem */ }
        }
        commandFaultCount++
        lastCommandFault = e
        val message = "COMMAND FAULT: ${e.javaClass.simpleName}: ${e.message} (commands cleared, subsystems halted)"
        try {
            RobotLog.ee("Robot", e, "$message (#$commandFaultCount)")
        } catch (_: Throwable) {
            // Host-side tests stub Android logging.
        }
        val now = clock.nanos()
        if (message != lastCommandFaultMessage || throttle(lastCommandFaultEventNs, now)) {
            lastCommandFaultMessage = message
            lastCommandFaultEventNs = now
            recordEvent(message)
        }
    }

    private fun recordTelemetryFault(t: Throwable) {
        telemetryFaultCount++
        if (telemetryFaultCount <= 5) {
            try {
                RobotLog.ee("Robot", t, "Telemetry fault contained (#$telemetryFaultCount)")
            } catch (_: Throwable) {
                // Host-side tests stub Android logging.
            }
        }
        val now = clock.nanos()
        if (throttle(lastTelemetryFaultEventNs, now)) {
            lastTelemetryFaultEventNs = now
            recordEvent("TELEMETRY FAULT: ${t.javaClass.simpleName}: ${t.message}")
        }
    }

    private fun recordRecorderFault(t: Throwable) {
        recorderFaultCount++
        try {
            RobotLog.ee("Robot", t, "Flight recorder fault contained")
        } catch (_: Throwable) {
            // Host-side tests stub Android logging.
        }
        try {
            flightRecorder?.close()
        } catch (_: Throwable) {
            // The recorder has already faulted; shutdown is best-effort.
        }
        flightRecorder = null
    }

    private fun throttle(lastNs: Long, now: Long): Boolean =
        lastNs == Long.MIN_VALUE || now - lastNs > EVENT_THROTTLE_NANOS

    /**
     * Stop every subsystem before callbacks or storage can block, then clear
     * Ivy without running end handlers (hardware is already stopped and must
     * stay that way), persist handoff state, and close the recorder. Runs once.
     * [afterHardwareStopped] can report a crash while motors are already off.
     */
    fun stop(afterHardwareStopped: () -> Unit = {}) {
        if (stopped) return
        stopped = true
        for (s in subsystems) {
            try { s.stop() } catch (_: Throwable) { /* try every subsystem */ }
        }
        try { afterHardwareStopped() } catch (_: Throwable) { /* preserve the original fault */ }
        try { Scheduler.reset() } catch (_: Throwable) { /* best-effort */ }
        try { recordEvent("stop") } catch (_: Throwable) { /* best-effort */ }
        if (loopCount > 0) {
            for (s in subsystems) {
                try { s.persistState() } catch (_: Throwable) { /* best-effort */ }
            }
        }
        try { closeFlightRecorder() } catch (_: Throwable) { /* best-effort */ }
        // INIT tuning must survive cancellation too; pose handoff above still requires an active loop.
        try { ConfigStore.persistIfDirty() } catch (_: Throwable) { /* preserve the original fault */ }
    }

    /** Loop frequency in Hz over the most recent tick. */
    val loopHz: Double
        get() = if (lastLoopNanos > 0) 1e9 / lastLoopNanos else 0.0

    private companion object {
        const val LOOP_OVERRUN_NANOS = 100_000_000L
        const val EVENT_THROTTLE_NANOS = 1_000_000_000L
    }
}
