package org.firstinspires.ftc.teamcode.core.command

/**
 * Command composition. Every group:
 *
 *  - requires the **union** of its children's requirements for its whole
 *    lifetime (children never touch the scheduler themselves)
 *  - keeps the default priority 0 — raise it explicitly with `setPriority`
 *    (the auton runner does) rather than inheriting from children
 *  - forwards interruption to whichever children are still running, so a
 *    cancelled group never leaves a child mid-flight
 */
object Groups {

    /** Children run one after another; the group completes after the last. */
    fun sequential(vararg commands: Command): CommandBuilder = SequentialGroup(commands.toList())

    /** Children all start together; the group completes when every one has. */
    fun parallel(vararg commands: Command): CommandBuilder = ParallelGroup(commands.toList())

    /**
     * Children all start together; the group completes the instant any one
     * does. The winner ends NATURALLY, the rest are interrupted.
     */
    fun race(vararg commands: Command): CommandBuilder = RaceGroup(commands.toList())

    /**
     * All children start together; when [deadline] completes, the remaining
     * children are interrupted.
     */
    fun deadline(deadline: Command, vararg others: Command): CommandBuilder =
        DeadlineGroup(deadline, others.toList())
}

private fun CommandBuilder.requireUnionOf(children: List<Command>): CommandBuilder {
    require(children.isNotEmpty()) { "group requires at least one command" }
    return requiring(children.flatMapTo(LinkedHashSet()) { it.requirements() })
}

private class SequentialGroup(private val children: List<Command>) : CommandBuilder() {
    private var index = 0

    init {
        requireUnionOf(children)
        setName("sequential(${children.size})")
        setStart {
            index = 0
            children[0].start()
        }
        setExecute {
            // Drain every child that finishes this tick (each still executes
            // at most once per tick): a zero-duration child — an instant, an
            // already-true waitUntil — must not cost the routine a dead tick.
            while (index < children.size) {
                val child = children[index]
                child.execute()
                if (!child.done()) break
                child.end(EndCondition.NATURALLY)
                index++
                if (index < children.size) children[index].start()
            }
        }
        setDone { index >= children.size }
        setEnd { condition ->
            if (condition != EndCondition.NATURALLY && index < children.size) {
                children[index].end(condition)
            }
        }
    }
}

private class ParallelGroup(private val children: List<Command>) : CommandBuilder() {
    private val finished = BooleanArray(children.size)

    init {
        requireUnionOf(children)
        setName("parallel(${children.size})")
        setStart {
            finished.fill(false)
            for (child in children) child.start()
        }
        setExecute {
            children.forEachIndexed { i, child ->
                if (finished[i]) return@forEachIndexed
                child.execute()
                if (child.done()) {
                    child.end(EndCondition.NATURALLY)
                    finished[i] = true
                }
            }
        }
        setDone { finished.all { it } }
        setEnd { condition ->
            if (condition == EndCondition.NATURALLY) return@setEnd
            children.forEachIndexed { i, child ->
                if (!finished[i]) child.end(condition)
            }
        }
    }
}

private class RaceGroup(private val children: List<Command>) : CommandBuilder() {
    private var winner = -1

    init {
        requireUnionOf(children)
        setName("race(${children.size})")
        setStart {
            winner = -1
            for (child in children) child.start()
        }
        setExecute {
            if (winner >= 0) return@setExecute
            for ((i, child) in children.withIndex()) {
                child.execute()
                if (child.done()) {
                    winner = i
                    break
                }
            }
        }
        setDone { winner >= 0 }
        setEnd { condition ->
            children.forEachIndexed { i, child ->
                when {
                    condition != EndCondition.NATURALLY -> child.end(condition)
                    i == winner -> child.end(EndCondition.NATURALLY)
                    else -> child.end(EndCondition.INTERRUPTED)
                }
            }
        }
    }
}

private class DeadlineGroup(
    private val deadline: Command,
    private val others: List<Command>,
) : CommandBuilder() {
    private val finished = BooleanArray(others.size)

    init {
        requireUnionOf(listOf(deadline) + others)
        setName("deadline(${1 + others.size})")
        setStart {
            finished.fill(false)
            deadline.start()
            for (child in others) child.start()
        }
        setExecute {
            deadline.execute()
            others.forEachIndexed { i, child ->
                if (finished[i]) return@forEachIndexed
                child.execute()
                if (child.done()) {
                    child.end(EndCondition.NATURALLY)
                    finished[i] = true
                }
            }
        }
        setDone { deadline.done() }
        setEnd { condition ->
            deadline.end(condition)
            val othersCondition =
                if (condition == EndCondition.NATURALLY) EndCondition.INTERRUPTED else condition
            others.forEachIndexed { i, child ->
                if (!finished[i]) child.end(othersCondition)
            }
        }
    }
}
