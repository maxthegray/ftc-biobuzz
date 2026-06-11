package org.firstinspires.ftc.teamcode.core.command

import java.util.Locale
import org.firstinspires.ftc.teamcode.core.util.Clock

/** Small command factories. Compose them with [Groups]. */
object Commands {

    /** Runs [action] once (in start) and completes on the next scheduler tick. */
    fun instant(action: () -> Unit): CommandBuilder = Command.build()
        .setName("instant")
        .setStart(action)
        .setDone { true }

    /**
     * Waits [ms] milliseconds on [clock]. Inject the robot's clock (e.g.
     * `robot.clock`) and waits become simulable in virtual time; the default
     * is the wall clock.
     */
    fun waitMs(ms: Double, clock: Clock = Clock.SYSTEM): CommandBuilder {
        require(ms.isFinite() && ms >= 0.0) { "waitMs requires a finite, non-negative duration" }
        val waitNanos = (ms * 1e6).toLong()
        var startedNs = 0L
        return Command.build()
            .setName("wait %.0f ms".format(Locale.US, ms))
            .setStart { startedNs = clock.nanos() }
            .setDone { clock.nanos() - startedNs >= waitNanos }
    }

    /** [waitMs] long-overload — keep both, op-modes pass literals of either type. */
    fun waitMs(ms: Long, clock: Clock = Clock.SYSTEM): CommandBuilder = waitMs(ms.toDouble(), clock)

    /** Completes once [condition] reads true (checked every tick). */
    fun waitUntil(condition: () -> Boolean): CommandBuilder = Command.build()
        .setName("waitUntil")
        .setDone(condition)

    /** Runs [action] every tick and never completes on its own. */
    fun infinite(action: () -> Unit): CommandBuilder = Command.build()
        .setName("infinite")
        .setExecute(action)

    /**
     * Defers command construction to schedule time: [supplier] runs in start,
     * and the produced command's lifecycle is forwarded. Use this when the
     * command depends on state only known when it actually runs (e.g. a path
     * built from the *current* pose).
     *
     * [requirements] must cover everything the supplied command will require —
     * the scheduler claims requirements at schedule time, before [supplier]
     * has run.
     */
    fun defer(vararg requirements: Any, supplier: () -> Command): CommandBuilder {
        var inner: Command? = null
        return Command.build()
            .setName("defer")
            .requiring(*requirements)
            .setStart { inner = supplier().also { it.start() } }
            .setExecute { inner?.execute() }
            .setDone { inner?.done() ?: true }
            .setEnd { condition ->
                inner?.end(condition)
                inner = null
            }
    }
}
