package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.behaviors.BlockedBehavior
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.behaviors.InterruptedBehavior
import org.firstinspires.ftc.teamcode.core.Clock

/**
 * Records [command]'s runs in the flight log under [name]. Schedule, cancel,
 * bind and compare the returned instance, not [command]: Ivy only sees the
 * wrapper. Wrapping an already logged command renames it instead of tracing
 * it twice.
 *
 * ```kotlin
 * fun grab(): Command = logged("Intake grab", Command.build().requiring(this).setDone { ballSeen })
 * ```
 */
fun logged(name: String, command: Command): Command =
    LoggedCommand(name, if (command is LoggedCommand) command.command else command)

/**
 * An Ivy [Command] that forwards every call to [command] (requirements,
 * priority and the three behaviours are read through on every call) and
 * reports its lifecycle to [CommandHistory].
 *
 * **Fault isolation.** An [Exception] thrown by [command] ends only this
 * command: it is recorded as FAIL, [onFault] makes its subsystems safe, then it
 * reports done and [command]'s end handler runs as for an interrupt. A parent
 * `sequential` moves on to its next step, so one broken step can't end an
 * auto. An [Error] (`TODO()`) still escapes and crashes the loop.
 *
 * Ivy 1.1.1 quirks it handles: `end` without a `start` (a later step of a
 * cancelled `sequential`, a cancelled queued command, a `lazy` forwarding to
 * its previous command) and a second `end` (a `deadline` child) record nothing;
 * `loop`/`repeat` restart the same instance, which is a new execution; a
 * SUSPENDED command resumes through `execute` without a new `start`; and
 * `Scheduler.reset` skips `end`, so [Robot][org.firstinspires.ftc.teamcode.core.Robot]
 * closes what is left with ABORT.
 */
class LoggedCommand internal constructor(val name: String, val command: Command) : Command {
    private var execution: CommandHistory.Execution? = null
    private var faulted = false

    override fun requirements(): MutableSet<Any> = command.requirements()
    override fun priority(): Int = command.priority()
    override fun interruptedBehavior(): InterruptedBehavior = command.interruptedBehavior()
    override fun conflictBehavior(): ConflictBehavior = command.conflictBehavior()
    override fun blockedBehavior(): BlockedBehavior = command.blockedBehavior()

    override fun start() {
        execution?.let { CommandHistory.abort(it, "restarted without an end") }
        faulted = false
        execution = CommandHistory.start(name)
        contained("start", Unit) { command.start() }
    }

    override fun execute() {
        if (faulted) return
        execution?.let(CommandHistory::resume)
        contained("execute", Unit) { command.execute() }
    }

    override fun done(): Boolean = faulted || contained("done", true) { command.done() }

    override fun end(endCondition: EndCondition) {
        val run = execution
        // A faulted command was cut short: let its cleanup run as for an interrupt.
        val condition = if (faulted && endCondition == EndCondition.NATURALLY) EndCondition.INTERRUPTED else endCondition
        faulted = false
        contained("end", Unit) { command.end(condition) }
        if (run != null) CommandHistory.ended(run, endCondition)
    }

    private inline fun <T> contained(phase: String, fallback: T, block: () -> T): T = try {
        block()
    } catch (e: Exception) {
        CommandHistory.fail(execution, name, phase, e)
        faulted = phase != "end"
        onFault(this, e)
        fallback
    } catch (t: Throwable) {
        CommandHistory.fail(execution, name, phase, t)
        throw t
    }

    override fun toString(): String = name

    companion object {
        /** Set by [Robot][org.firstinspires.ftc.teamcode.core.Robot]: halt the command's subsystems and record the fault. */
        internal var onFault: (LoggedCommand, Exception) -> Unit = { _, _ -> }
    }
}

/**
 * The one record of traced command executions for the current [Robot][org.firstinspires.ftc.teamcode.core.Robot].
 * Like Ivy's scheduler it is static and single-threaded (the robot loop).
 *
 * The open executions are the truth; every lifecycle change is timestamped on
 * the robot clock when it happens, together with the active set it leaves
 * behind, and queued. [FlightRecorder] writes the queue to `commands/events`
 * and `commands/active` each loop and at close, so tracing never does I/O.
 * Both the open set and the queue are bounded; anything dropped is counted in
 * [lost] and logged as incomplete history. A tracing failure is counted the
 * same way and never reaches the command.
 */
internal object CommandHistory {
    class Execution(val id: Long, val name: String) {
        var open = true
        var suspended = false
    }

    class Record(val nanos: Long, val text: String, val active: String)

    const val MAX_OPEN = 64
    const val MAX_PENDING = 512
    private const val MAX_DETAIL = 160

    private var clock: Clock = Clock.SYSTEM
    private var nextId = 1L
    private val open = LinkedHashMap<Long, Execution>()
    private val pending = ArrayDeque<Record>()
    private var recording = false
    private var lastFailure: Throwable? = null
    private var lastFailureId = 0L

    /** Records dropped since [reset]: history after the first loss is incomplete. */
    var lost = 0L
        private set

    /** A new op-mode: forget everything without recording it. */
    fun reset(clock: Clock) {
        for (e in open.values) e.open = false
        open.clear()
        pending.clear()
        this.clock = clock
        nextId = 1
        lost = 0
        recording = false
        lastFailure = null
    }

    fun startRecording() {
        recording = true
    }

    fun stopRecording() {
        recording = false
        pending.clear()
    }

    fun start(name: String): Execution? = traced(null) {
        val now = clock.nanos()
        if (open.size >= MAX_OPEN) {
            lost++
            return@traced null
        }
        val run = Execution(nextId++, name)
        open[run.id] = run
        emit(now, "START #${run.id} $name")
        run
    }

    fun ended(run: Execution, condition: EndCondition) = traced(Unit) {
        if (!run.open) return@traced
        when (condition) {
            EndCondition.NATURALLY -> close(run, "FINISH", "")
            EndCondition.INTERRUPTED -> close(run, "INTERRUPT", "")
            EndCondition.SUSPENDED -> if (!run.suspended) {
                run.suspended = true
                emit(clock.nanos(), "SUSPEND #${run.id} ${run.name}")
            }
        }
    }

    fun resume(run: Execution) = traced(Unit) {
        if (!run.open || !run.suspended) return@traced
        run.suspended = false
        emit(clock.nanos(), "RESUME #${run.id} ${run.name}")
    }

    /** [run] (null or closed: outside a traced execution) threw [t] in [phase]. */
    fun fail(run: Execution?, name: String, phase: String, t: Throwable) = traced(Unit) {
        val origin = if (t === lastFailure) " (from #$lastFailureId)" else ""
        val detail = " in $phase: ${t.javaClass.simpleName}: ${t.message}".take(MAX_DETAIL) + origin
        if (run != null && run.open) {
            if (lastFailure !== t) {
                lastFailure = t
                lastFailureId = run.id
            }
            close(run, "FAIL", detail)
        } else {
            emit(clock.nanos(), "FAIL #- $name$detail")
        }
    }

    fun abort(run: Execution, reason: String) = traced(Unit) {
        if (run.open) close(run, "ABORT", ": $reason")
    }

    /** Bookkeeping after Ivy was reset without end handlers: close every open execution. */
    fun abortAll(reason: String) = traced(Unit) {
        for (run in open.values.toList()) close(run, "ABORT", ": $reason")
    }

    /** The oldest queued record, or null. */
    fun poll(): Record? = pending.removeFirstOrNull()

    private fun close(run: Execution, verb: String, detail: String) {
        run.open = false
        open.remove(run.id)
        emit(clock.nanos(), "$verb #${run.id} ${run.name}$detail")
    }

    private fun emit(nanos: Long, text: String) {
        if (!recording) return
        if (pending.size >= MAX_PENDING) {
            lost++
            return
        }
        val active = open.values.joinToString("\n") { "#${it.id} ${it.name}" + if (it.suspended) " (suspended)" else "" }
        pending.addLast(Record(nanos, text, active))
    }

    private inline fun <T> traced(fallback: T, block: () -> T): T = try {
        block()
    } catch (_: Throwable) {
        lost++
        fallback
    }
}
