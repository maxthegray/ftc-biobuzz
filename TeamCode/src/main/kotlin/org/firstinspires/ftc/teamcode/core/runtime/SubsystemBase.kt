package org.firstinspires.ftc.teamcode.core.runtime

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.qualcomm.robotcore.hardware.HardwareMap
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
     * Ivy command [Robot] schedules whenever this subsystem is free. Assigning
     * one sets its [ConflictBehavior] to CANCEL (through a [LoggedCommand] to the
     * builder it wraps), so a default never preempts an explicit command of
     * equal priority; explicit commands at [CommandPriorities.DEFAULT] or above
     * still preempt the default.
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

    /**
     * Subsystem type that must already be registered before this one.
     * [Robot.register] enforces it, turning a silent ordering bug (e.g. a
     * localizer sampling pose history before the drive has updated the
     * follower) into an init-time error.
     */
    open val registerAfter: Class<out SubsystemBase>? get() = null

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
     */
    open fun logState(log: StateLog) {}

    /**
     * A command threw. [Robot] has already cleared Ivy's scheduler (no end
     * handlers run) and calls this on every subsystem: put actuators in a safe
     * state. Default commands resume on the next tick. Never throw.
     */
    open fun onCommandFault() {}

    /**
     * Called once at the end of the op-mode, before diagnostics and storage.
     * Zero actuators first; avoid logging or storage I/O here. Never throw.
     */
    open fun stop() {}

    override fun toString(): String = "Subsystem($name)"
}

/**
 * Shared command priority ladder. Higher interrupts lower; gaps left so season
 * code can slot levels in between without renumbering.
 *
 *  - [DEFAULT] — subsystem default commands; quietly lose to explicit work.
 *  - [AUTON_ROUTINE] — autonomous routines and teleop auto-assists
 *    (auto-aim, auto-align). Interrupt defaults only.
 *  - [DRIVER_ACTION] — direct driver-triggered actions. Deliberately above
 *    [AUTON_ROUTINE] so a driver input always wins over an assist.
 *  - [DRIVER_OVERRIDE] — panic / manual-override bindings that must preempt
 *    everything (e.g. "stop the lift NOW").
 */
object CommandPriorities {
    const val DEFAULT: Int = 0
    const val AUTON_ROUTINE: Int = 10
    const val DRIVER_ACTION: Int = 20
    const val DRIVER_OVERRIDE: Int = 30
}
