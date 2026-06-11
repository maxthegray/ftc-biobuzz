package org.firstinspires.ftc.teamcode.core.command

/**
 * Instance-scoped command scheduler. One per
 * [org.firstinspires.ftc.teamcode.core.runtime.Robot] — no global state, so
 * op-modes, hot reloads, and host tests can never leak commands into each
 * other.
 *
 * Scheduling semantics (deliberately matching the Ivy 1.0 behavior this
 * replaced, so the framework's priority ladder keeps working unchanged):
 *
 *  - A command starts if every requirement is free.
 *  - It is **blocked** (silently dropped) if any requirement is held by a
 *    *strictly higher*-priority command.
 *  - Otherwise the equal/lower-priority holders are interrupted and it starts.
 *
 * Improvements over Ivy:
 *
 *  - **End handlers always run** — on natural completion, cancel, [reset],
 *    preemption, and faults — so cleanup logic can live in the command
 *    instead of subsystem safety nets.
 *  - **Faults are per-command.** An exception from one command's lifecycle
 *    ends *that* command with [EndCondition.FAULTED], releases its
 *    requirements, and reports through [faultHandler]; every other running
 *    command keeps going. With no handler installed the exception propagates
 *    (auton fail-fast).
 *  - Native introspection ([runningCommands]) instead of reflection.
 */
class Scheduler {

    private val running = LinkedHashSet<Command>()
    private val activeRequirements = HashMap<Any, Command>()

    /**
     * Receives (command, exception) for every contained lifecycle fault.
     * Null (the default) rethrows instead — the fail-fast mode.
     * [org.firstinspires.ftc.teamcode.core.runtime.Robot] installs its
     * policy here; the handler may itself rethrow to escalate.
     */
    var faultHandler: ((Command, Throwable) -> Unit)? = null

    /**
     * Schedule [command]. Returns true if it is running afterwards (including
     * the already-scheduled no-op case); false if it was blocked by a
     * higher-priority holder or its [Command.start] faulted.
     */
    fun schedule(command: Command): Boolean {
        if (command in running) return true
        val holders = command.requirements().mapNotNullTo(HashSet()) { activeRequirements[it] }
        if (holders.any { it.priority() > command.priority() }) return false
        for (holder in holders) {
            remove(holder)
            endReported(holder, EndCondition.INTERRUPTED)
        }
        running += command
        for (requirement in command.requirements()) activeRequirements[requirement] = command
        try {
            command.start()
        } catch (t: Throwable) {
            remove(command)
            endSwallowed(command, EndCondition.FAULTED)
            report(command, t)
            return false
        }
        return true
    }

    /** Cancel [command] if it is running; its end handler runs with INTERRUPTED. */
    fun cancel(command: Command) {
        if (command !in running) return
        remove(command)
        endReported(command, EndCondition.INTERRUPTED)
    }

    fun isScheduled(command: Command): Boolean = command in running

    fun isRunning(command: Command): Boolean = command in running

    /**
     * One scheduler tick: every running command executes, then its done
     * condition is checked. Call exactly once per main-loop tick.
     */
    fun execute() {
        for (command in running.toList()) {
            if (command !in running) continue // ended by a sibling this tick
            try {
                command.execute()
                if (command.done()) {
                    remove(command)
                    endReported(command, EndCondition.NATURALLY)
                }
            } catch (t: Throwable) {
                remove(command)
                endSwallowed(command, EndCondition.FAULTED)
                report(command, t)
            }
        }
    }

    /**
     * Interrupt every running command. End handlers run (best-effort: an end
     * handler that throws is swallowed — reset is the shutdown path and every
     * command must get its chance to clean up).
     */
    fun reset() {
        for (command in running.toList()) {
            remove(command)
            endSwallowed(command, EndCondition.INTERRUPTED)
        }
    }

    /** Currently running commands, in schedule order. */
    fun runningCommands(): List<Command> = running.toList()

    /** `toString()` of each running command, for telemetry and flight logs. */
    fun runningCommandNames(): List<String> = running.map { it.toString() }

    private fun remove(command: Command) {
        running -= command
        activeRequirements.entries.removeAll { it.value === command }
    }

    /** Run the end handler; a throwing end handler is itself a fault. */
    private fun endReported(command: Command, condition: EndCondition) {
        try {
            command.end(condition)
        } catch (t: Throwable) {
            report(command, t)
        }
    }

    private fun endSwallowed(command: Command, condition: EndCondition) {
        try {
            command.end(condition)
        } catch (_: Throwable) {
            // Best-effort: the command is already being torn down.
        }
    }

    private fun report(command: Command, t: Throwable) {
        val handler = faultHandler ?: throw t
        handler(command, t)
    }
}
