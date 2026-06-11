package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.Command

/**
 * Base class every subsystem extends.
 *
 * A subsystem owns a slice of the robot (drive, intake, shooter, lift…) and
 * presents a safe, high-level API. The main loop is single-threaded: the
 * OpMode calls [periodic] on every subsystem once per tick, after the Lynx
 * bulk read but before the command scheduler tick.
 *
 * Subsystems double as command "requirements" — pass `this` to
 * `CommandBuilder.requiring(...)` and the scheduler will automatically
 * resolve conflicts between commands that touch the same hardware.
 */
abstract class SubsystemBase(val name: String) {

    /**
     * Command the robot should keep scheduled whenever this subsystem is free.
     * Defaults use priority [CommandPriorities.DEFAULT]; explicit driver or
     * auton actions should use priority 10 and require the same subsystem.
     */
    var defaultCommand: Command? = null

    /**
     * Subsystem type that must already be registered before this one.
     * [Robot.register] enforces it, turning a silent ordering bug (e.g. a
     * localizer sampling pose history before the drive has updated the
     * follower) into an init-time error.
     */
    open val registerAfter: Class<out SubsystemBase>? get() = null

    /**
     * Called exactly once when the OpMode initialises. Resolve hardware,
     * zero encoders, apply motor directions. Failures should throw — the
     * [Robot] catches them and reports via telemetry.
     */
    open fun init(hardwareMap: HardwareMap) {}

    /**
     * Called every main-loop tick, after the bulk read completes and before
     * the scheduler executes commands. Use this for pure reads / state
     * updates — not for commanding actuators. Command hardware from commands
     * so the scheduler can arbitrate conflicts.
     */
    open fun periodic() {}

    /**
     * Called every main-loop tick after [periodic] and the scheduler tick.
     * Useful for writing the final motor power / servo position decided by
     * whichever command is currently running.
     */
    open fun writeHardware() {}

    /**
     * Persist subsystem state that should survive op-mode handoff. Called
     * during [Robot.stop] only after at least one real loop has run, so an
     * init-cancelled op-mode cannot overwrite useful state with defaults.
     */
    open fun persistState() {}

    /**
     * Short health string for Driver Station / Panels telemetry, or null if
     * the subsystem has nothing useful to report this tick.
     */
    open fun health(): String? = null

    /**
     * Write this subsystem's state channels into the flight log. Called once
     * per tick by the flight recorder (when one is running); channel names
     * are automatically prefixed with `<name>/`. Log what tuning and
     * post-match triage need — goals, setpoints, measurements, outputs.
     */
    open fun logState(log: org.firstinspires.ftc.teamcode.core.logging.StateLog) {}

    /**
     * Called after [Robot] contains a fault from a command that *required
     * this subsystem* (teleop only). The faulting command has already been
     * ended (its end handler ran, best-effort) — this is the safety net for
     * when that end handler is itself the buggy code: put actuators in a
     * safe state here — break a path follow, hold a lift in place. The
     * subsystem's default command resumes on the next tick. Never throw.
     */
    open fun onCommandFault() {}

    /**
     * Called once at the end of the OpMode. Zero motors, stop threads,
     * release any resources. Never throw from here.
     */
    open fun stop() {}

    override fun toString(): String = "Subsystem($name)"
}
