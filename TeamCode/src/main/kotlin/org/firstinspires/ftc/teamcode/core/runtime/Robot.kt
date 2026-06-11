package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Scheduler
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.core.logging.FlightRecorder
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.firstinspires.ftc.teamcode.core.util.Clock
import org.firstinspires.ftc.teamcode.core.util.GamepadEx

/**
 * Robot is the single source of truth for the robot's hardware + subsystems
 * + command scheduler for the lifetime of one OpMode.
 *
 * Responsibilities:
 *  - Own the list of [SubsystemBase]s
 *  - Put every [LynxModule] into MANUAL bulk read mode and clear the cache
 *    exactly once per main-loop tick
 *  - Drive Ivy's [Scheduler] each tick
 *  - Provide a consistent lifecycle: init → start → loop (read/periodic/tick/write) → stop
 *
 * [OpModeBase] handles the actual OpMode plumbing; you rarely construct
 * Robot yourself.
 */
class Robot(
    val hardwareMap: HardwareMap,
    private val clock: Clock = Clock.SYSTEM,
) {

    private val subsystems = mutableListOf<SubsystemBase>()
    val bulkRead = BulkReadManager(hardwareMap)

    var alliance: Alliance = Alliance.RED

    /**
     * When true, an exception thrown from the command layer (trigger polling,
     * default scheduling, `Scheduler.execute`) is *contained* instead of
     * killing the op-mode: the scheduler is cleared, every subsystem gets
     * [SubsystemBase.onCommandFault] to safe its actuators, and the loop
     * keeps running — default commands (including teleop drive) resume on
     * the next tick.
     *
     * Off by default. Teleop op-modes turn it on (a dead robot for the rest
     * of a match is strictly worse than one mechanism going limp); auton
     * leaves it off, where driving on silently-wrong state is the greater
     * danger. Subsystem `periodic()`/`writeHardware()` and the op-mode's
     * `onLoop()` always fail fast — they have no clean recovery story.
     */
    var containCommandFaults: Boolean = false

    /** Number of contained command faults this op-mode. Surfaced in health telemetry. */
    var commandFaultCount: Int = 0
        private set

    /** The most recent contained fault, for telemetry and post-match triage. */
    var lastCommandFault: Throwable? = null
        private set

    /** Monotonic tick counter, useful for log throttling. */
    var loopCount: Long = 0
        private set

    /** Wall-clock duration of the most recent loop in nanoseconds. */
    var lastLoopNanos: Long = 0
        private set

    /** Per-phase breakdown of the most recent loop. Overwritten in place each tick. */
    val profile = LoopProfile()

    private var lastTickEndNs: Long = 0
    private var flightRecorder: FlightRecorder? = null

    private var initialized = false

    /** Subsystems whose default command has already been evented this op-mode. */
    private val defaultEventSent = HashSet<SubsystemBase>()
    private var lastOverrunEventNs = Long.MIN_VALUE

    /**
     * Register a subsystem. Must happen before [init] (i.e. in the op-mode's
     * `configure()`) — a subsystem registered later would silently never get
     * its `init`, so this throws instead. Order matters only for tie-breaking
     * in telemetry output.
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
    ) {
        flightRecorder = FlightRecorder.open(opModeClassName, driver, operator, batteryVoltage)
    }

    fun recordEvent(message: String) {
        flightRecorder?.event(message)
        recentEvents.addLast("[loop $loopCount] $message")
        while (recentEvents.size > RECENT_EVENT_LIMIT) recentEvents.removeFirst()
    }

    private val recentEvents = ArrayDeque<String>()

    /** The last few recorded events, oldest first — crash reports include these. */
    fun recentEvents(): List<String> = recentEvents.toList()

    fun closeFlightRecorder() {
        flightRecorder?.close()
        flightRecorder = null
    }

    /**
     * Initialise every registered subsystem. Any exception from a subsystem's
     * init is rethrown after logging so the OpMode fails loudly rather than
     * silently running with half-initialised hardware.
     */
    fun init() {
        initialized = true
        bulkRead.init()
        Scheduler.reset()
        for (s in subsystems) s.init(hardwareMap)
    }

    /**
     * Called once at the moment the OpMode actually starts running (after
     * init but before the first loop tick). Resets the loop timer so the
     * first [lastLoopNanos] reading isn't skewed by init time.
     */
    fun start() {
        lastTickEndNs = clock.nanos()
        loopCount = 0
    }

    /**
     * Single main-loop tick. Order is fixed and tuned for correctness:
     *  1. Clear Lynx bulk caches so all reads this tick return fresh data
     *  2. [SubsystemBase.periodic] — pure reads, state updates
     *  3. [input] — gamepad edge detection + trigger polling, so bindings
     *     (including sensor-backed triggers) react to *this* tick's data
     *  4. [control] — OpMode code reacts to fresh state and schedules commands
     *  5. Default commands — idle subsystem behavior is scheduled if free
     *  6. [Scheduler.execute] — commands tick, deciding motor/servo targets
     *  7. [SubsystemBase.writeHardware] — flush those targets to hardware
     *  8. [telemetry] — publish + (throttled) transmit, measured like any
     *     other phase instead of hiding in loop overhead
     *
     * Note: because [telemetry] runs before this tick's total is computed,
     * any loop-total it publishes is the *previous* tick's. Per-phase numbers
     * are current.
     *
     * Returns the tick duration in nanoseconds.
     */
    fun loop(
        input: () -> Unit = {},
        control: () -> Unit = {},
        telemetry: () -> Unit = {},
    ): Long {
        var phaseStart = clock.nanos()

        bulkRead.clearCaches()
        phaseStart = mark(phaseStart) { profile.clearCachesNanos = it }

        for (s in subsystems) s.periodic()
        phaseStart = mark(phaseStart) { profile.periodicNanos = it }

        commandPhase { input() }
        phaseStart = mark(phaseStart) { profile.inputNanos = it }

        control()
        phaseStart = mark(phaseStart) { profile.controlNanos = it }

        commandPhase {
            for (s in subsystems) {
                val default = s.defaultCommand
                if (default != null && !Scheduler.isScheduled(default)) {
                    Scheduler.schedule(default)
                    // Event only the first resume (and the first after a fault):
                    // logging every post-action resume floods the recent-events
                    // ring that crash reports rely on.
                    if (Scheduler.isScheduled(default) && defaultEventSent.add(s)) {
                        recordEvent("schedule default ${s.name}: $default")
                    }
                }
            }
            Scheduler.execute()
        }
        phaseStart = mark(phaseStart) { profile.schedulerNanos = it }

        for (s in subsystems) s.writeHardware()
        phaseStart = mark(phaseStart) { profile.writeHardwareNanos = it }

        telemetry()
        val afterTelemetry = clock.nanos()
        profile.telemetryNanos = afterTelemetry - phaseStart

        val recorder = flightRecorder
        if (recorder == null) {
            profile.recordNanos = 0
        } else {
            // Measure twice on purpose: the first reading is what gets logged
            // (the recorder can't time its own final write), the second adds
            // that write's cost so the profile accounts for the full phase.
            val recordStart = clock.nanos()
            recorder.record(this)
            recorder.recordRecorderNanos(clock.nanos() - recordStart)
            profile.recordNanos = clock.nanos() - recordStart
        }
        val now = clock.nanos()

        lastLoopNanos = now - lastTickEndNs
        profile.totalNanos = lastLoopNanos
        lastTickEndNs = now
        loopCount++

        // Watchdog: a grossly slow tick is a competition symptom worth a log
        // event even if nobody is watching the loop telemetry. Throttled so a
        // sustained stall doesn't flood the recent-events ring.
        if (lastLoopNanos > LOOP_OVERRUN_NANOS &&
            (lastOverrunEventNs == Long.MIN_VALUE || now - lastOverrunEventNs > OVERRUN_EVENT_THROTTLE_NANOS)
        ) {
            lastOverrunEventNs = now
            recordEvent("LOOP OVERRUN: ${lastLoopNanos / 1_000_000} ms")
        }
        return lastLoopNanos
    }

    /**
     * One init-phase tick: fresh bulk read + subsystem reads, nothing else.
     * No commands run and nothing is written to hardware — the robot must
     * not move before start. Gamepad edge updates happen in [OpModeBase]'s
     * init loop with trigger polling disabled, so bindings wired in
     * `configure()` cannot start commands early.
     */
    fun initTick() {
        bulkRead.clearCaches()
        for (s in subsystems) s.periodic()
    }

    /** Record the duration of the just-finished phase and return the new phase start. */
    private inline fun mark(start: Long, assign: (Long) -> Unit): Long {
        val now = clock.nanos()
        assign(now - start)
        return now
    }

    /** Run a command-layer phase, containing faults if [containCommandFaults] is on. */
    private inline fun commandPhase(block: () -> Unit) {
        if (!containCommandFaults) {
            block()
            return
        }
        try {
            block()
        } catch (t: Throwable) {
            handleCommandFault(t)
        }
    }

    private fun handleCommandFault(t: Throwable) {
        commandFaultCount++
        lastCommandFault = t
        try {
            RobotLog.ee("Robot", t, "Command fault contained (#$commandFaultCount)")
        } catch (_: Throwable) {
            // Host-side tests stub Android logging.
        }
        recordEvent("COMMAND FAULT: ${t.javaClass.simpleName}: ${t.message}")
        // Re-arm the default-resume events so the post-fault recovery shows in the log.
        defaultEventSent.clear()
        // Scheduler.reset() skips end handlers, so subsystems must safe their
        // own actuators below. Default commands reschedule on the next tick.
        Scheduler.reset()
        for (s in subsystems) {
            try { s.onCommandFault() } catch (_: Throwable) { /* best-effort */ }
        }
    }

    /**
     * Shutdown everything. Ivy's `Scheduler.reset()` clears command state without
     * calling command end handlers, so critical hardware cleanup belongs in each
     * subsystem's [SubsystemBase.stop]. Exceptions are swallowed so all subsystems
     * get a chance to clean up.
     */
    fun stop() {
        recordEvent("stop")
        if (loopCount > 0) {
            for (s in subsystems) {
                try { s.persistState() } catch (_: Throwable) { /* best-effort */ }
            }
        }
        Scheduler.reset()
        for (s in subsystems) {
            try { s.stop() } catch (_: Throwable) { /* best-effort */ }
        }
        closeFlightRecorder()
    }

    /** Average loop frequency in Hz over the most recent tick. */
    val loopHz: Double
        get() = if (lastLoopNanos > 0) 1e9 / lastLoopNanos else 0.0

    private companion object {
        const val RECENT_EVENT_LIMIT = 20
        const val LOOP_OVERRUN_NANOS = 100_000_000L
        const val OVERRUN_EVENT_THROTTLE_NANOS = 1_000_000_000L
    }
}
