package org.firstinspires.ftc.teamcode.core.command

/** Why a command's [Command.end] was called. */
enum class EndCondition {
    /** [Command.done] returned true; the command completed on its own. */
    NATURALLY,

    /** Cancelled, preempted by a higher/equal-priority command, or reset. */
    INTERRUPTED,

    /** The command's own lifecycle code threw; see [Scheduler.faultHandler]. */
    FAULTED,
}

/**
 * A unit of robot behavior with a strict lifecycle, owned by a [Scheduler]:
 *
 *  - [start] — once, when scheduled
 *  - [execute] — every scheduler tick while running
 *  - [done] — checked after each [execute]; true ends the command
 *  - [end] — exactly once per run, with the [EndCondition]; **always runs**,
 *    including on cancel, scheduler reset, and contained faults
 *
 * [requirements] are opaque tokens (by convention the
 * [org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase]s the command
 * actuates); the scheduler guarantees one running command per requirement.
 * [priority] arbitrates conflicts — see
 * [org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities] for the
 * framework ladder.
 *
 * Commands are reusable: a single instance can be scheduled repeatedly
 * (trigger bindings hold one instance forever), so [start] must fully reset
 * any per-run state.
 */
interface Command {
    fun requirements(): Set<Any>
    fun priority(): Int
    fun start()
    fun execute()
    fun done(): Boolean
    fun end(endCondition: EndCondition)

    companion object {
        /** Entry point for the builder API: `Command.build().setStart { … }`. */
        @JvmStatic
        fun build(): CommandBuilder = CommandBuilder()
    }
}

/**
 * Fluent [Command] assembled from lambdas. Also the base class for composite
 * commands ([Groups]) — subclasses configure their lifecycle in `init`.
 *
 * Defaults: no requirements, priority 0, every hook a no-op, [done] false
 * (runs until cancelled). Give framework-built commands a [setName] so the
 * flight log's `commands/running` channel reads as a timeline, not hashes.
 */
open class CommandBuilder : Command {
    private val requirements = LinkedHashSet<Any>()
    private var priority = 0
    private var name: String? = null
    private var startAction: () -> Unit = {}
    private var executeAction: () -> Unit = {}
    private var doneCondition: () -> Boolean = { false }
    private var endAction: (EndCondition) -> Unit = {}

    final override fun requirements(): Set<Any> = requirements
    final override fun priority(): Int = priority
    final override fun start() = startAction()
    final override fun execute() = executeAction()
    final override fun done(): Boolean = doneCondition()
    final override fun end(endCondition: EndCondition) = endAction(endCondition)

    fun setStart(action: () -> Unit): CommandBuilder {
        startAction = action
        return this
    }

    fun setExecute(action: () -> Unit): CommandBuilder {
        executeAction = action
        return this
    }

    fun setDone(condition: () -> Boolean): CommandBuilder {
        doneCondition = condition
        return this
    }

    fun setEnd(action: (EndCondition) -> Unit): CommandBuilder {
        endAction = action
        return this
    }

    fun requiring(vararg requirements: Any): CommandBuilder {
        this.requirements.addAll(requirements)
        return this
    }

    fun requiring(requirements: Collection<Any>): CommandBuilder {
        this.requirements.addAll(requirements)
        return this
    }

    fun setPriority(priority: Int): CommandBuilder {
        this.priority = priority
        return this
    }

    fun setName(name: String): CommandBuilder {
        this.name = name
        return this
    }

    override fun toString(): String = name ?: "command"
}
