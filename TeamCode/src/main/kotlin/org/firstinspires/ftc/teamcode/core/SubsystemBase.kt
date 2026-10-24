package org.firstinspires.ftc.teamcode.core

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.logging.AutomaticStateLog
import org.firstinspires.ftc.teamcode.core.logging.LoggedCommand
import org.firstinspires.ftc.teamcode.core.logging.StateLog

/**
 * Base class every subsystem extends.
 *
 * A subsystem owns a slice of the robot (drive, intake, shooter, lift…) and
 * presents a safe, high-level API. The main loop is single-threaded: [Robot]
 * calls [periodic] on every subsystem once per tick, after the Lynx bulk read
 * but before Ivy's scheduler runs.
 *
 * Subsystems double as Ivy command requirements — pass `this` to
 * `CommandBuilder.requiring(...)` and the scheduler resolves conflicts between
 * commands that touch the same hardware.
 */
abstract class SubsystemBase(val name: String) {

    /**
     * Ivy command [Robot] schedules whenever this subsystem is free. Any other
     * command that requires the subsystem replaces it, and it comes back when
     * that command ends. (Assigning one sets its [ConflictBehavior] to CANCEL,
     * which is what stops it replacing a running command.)
     */
    var defaultCommand: Command? = null
        set(value) {
            field = value?.also(::cancelOnConflict)
        }

    private fun cancelOnConflict(command: Command) {
        when (command) {
            is CommandBuilder -> command.setConflictBehavior(ConflictBehavior.CANCEL)
            is LoggedCommand -> cancelOnConflict(command.command)
            else -> require(command.conflictBehavior() == ConflictBehavior.CANCEL) {
                "A default command must use ConflictBehavior.CANCEL"
            }
        }
    }

    /** Called once when the op-mode initialises. Resolve hardware here; failures should throw. */
    open fun init(hardwareMap: HardwareMap) {}

    /** Every tick, before commands run. Read sensors and update state; never command actuators. */
    open fun periodic() {}

    /** Init-loop reads, without commands or actuator writes. Defaults to [periodic]. */
    open fun initPeriodic() = periodic()

    /** Every tick after commands run. Flush the targets the running command decided. */
    open fun writeHardware() {}

    /** Short health string for Driver Station / Panels telemetry, or null. */
    open fun health(): String? = null

    /**
     * Write this subsystem's flight-log channels. Called at most 100 times a
     * second by the flight recorder; channel names are prefixed with `<name>/`.
     * Public scalar getters declared on this class are automatic; override for
     * arrays or specific channel names. Inherited, internal and publicly writable state is skipped.
     */
    open fun logState(log: StateLog) = AutomaticStateLog.record(this, log)

    /**
     * Something threw: a command using this subsystem, or an unlogged command
     * (then [Robot] has cleared Ivy's scheduler and calls this on every
     * subsystem). Put actuators in a safe state. Default commands resume on
     * the next tick. Never throw.
     */
    open fun onCommandFault() {}

    /**
     * This subsystem's writeHardware() threw, so its outputs may be stuck at
     * their last values. Zero them now, but leave the running command alone
     * so a one-tick glitch costs one tick. Defaults to [onCommandFault].
     * Never throw.
     */
    open fun onWriteFault() = onCommandFault()

    /**
     * Called once at the end of the op-mode, before diagnostics and storage.
     * Zero actuators first; avoid logging or storage I/O here. Never throw.
     */
    open fun stop() {}

    override fun toString(): String = "Subsystem($name)"
}

