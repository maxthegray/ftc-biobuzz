package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder

/**
 * Ivy's `Commands.waitMs` with a monotonic clock. Ivy 1.1.1 times `waitMs`
 * with `System.currentTimeMillis()`, which jumps when the Control Hub's wall
 * clock is set: a jump forward ends every wait at once, a jump back stalls
 * it. This is the same command timed by [clock] ([Clock.SYSTEM] is
 * `System.nanoTime()`). The timer restarts at every start, and it composes
 * (`race`, `sequential`, …) and cancels like any Ivy command.
 */
fun monotonicWaitMs(milliseconds: Double, clock: Clock = Clock.SYSTEM): CommandBuilder {
    require(milliseconds.isFinite() && milliseconds >= 0.0) { "wait must be finite and non-negative: $milliseconds" }
    val durationNs = (milliseconds * 1e6).toLong()
    var startNs = 0L
    return Command.build()
        .setStart { startNs = clock.nanos() }
        .setDone { clock.nanos() - startNs >= durationNs }
}
