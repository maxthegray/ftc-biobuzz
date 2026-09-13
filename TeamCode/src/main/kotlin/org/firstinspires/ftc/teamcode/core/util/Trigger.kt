package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import java.util.function.BooleanSupplier

/**
 * A boolean condition — a gamepad button, an analog trigger past a threshold,
 * a sensor state — bound to Ivy [Command]s. Wire bindings once in
 * `configure()`:
 *
 * ```kotlin
 * driver.button(GamepadEx.Button.A).onTrue(intake.grab())
 * driver.button(GamepadEx.Button.LEFT_BUMPER).whileTrue(intake.eject())
 * driver.trigger { driver.rightTrigger > 0.5 }.whileTrue(drive.slowMode())
 * (driver.button(GamepadEx.Button.BACK) and driver.button(GamepadEx.Button.Y))
 *     .onTrue(resetHeading())
 * ```
 *
 * Create one through [GamepadEx.button] / [GamepadEx.trigger], or by composing
 * existing triggers with [and] / [or] / [not]. The owning [GamepadEx] samples
 * every trigger once per loop in [GamepadEx.update], before Ivy's scheduler
 * runs. Composed triggers reuse their operands' samples.
 *
 * Compose triggers from the same [GamepadEx] host: a cross-gamepad
 * composition is polled by the left-hand trigger's host, so the other gamepad
 * may still be on its previous loop sample.
 */
class Trigger internal constructor(
    private val host: GamepadEx,
    private val condition: BooleanSupplier,
) {
    private var lastState = false
    private val bindings = mutableListOf<(prev: Boolean, curr: Boolean) -> Unit>()

    /** Schedule [command] on each rising edge. */
    fun onTrue(command: Command): Trigger = bind { prev, curr ->
        if (!prev && curr) Scheduler.schedule(command)
    }

    /** Schedule [command] on each falling edge. */
    fun onFalse(command: Command): Trigger = bind { prev, curr ->
        if (prev && !curr) Scheduler.schedule(command)
    }

    /** Schedule [command] on the rising edge and cancel it on the falling edge. */
    fun whileTrue(command: Command): Trigger = bind { prev, curr ->
        if (!prev && curr) {
            Scheduler.schedule(command)
        } else if (prev && !curr) {
            Scheduler.cancel(command)
        }
    }

    /** On each rising edge, cancel [command] if it is scheduled, otherwise schedule it. */
    fun toggleOnTrue(command: Command): Trigger = bind { prev, curr ->
        if (!prev && curr) {
            if (Scheduler.isScheduled(command)) Scheduler.cancel(command) else Scheduler.schedule(command)
        }
    }

    /** Active only while both this and [other] are active. */
    infix fun and(other: Trigger): Trigger = host.trigger { read() && other.read() }

    /** Active while either this or [other] is active. */
    infix fun or(other: Trigger): Trigger = host.trigger { read() || other.read() }

    /** Active exactly when this trigger is not. Also usable as `!trigger`. */
    operator fun not(): Trigger = host.trigger { !read() }

    private fun bind(binding: (prev: Boolean, curr: Boolean) -> Unit): Trigger {
        host.requireBindingsUnlocked()
        bindings += binding
        return this
    }

    /** This trigger's sample for the current host tick. */
    internal fun read(): Boolean = lastState

    /**
     * Sample without firing bindings. [GamepadEx.lockBindings] primes every
     * trigger at start so a button held through init doesn't fire as a
     * rising edge on the first real poll.
     */
    internal fun prime() {
        lastState = condition.asBoolean
    }

    /** Sample the condition and fire any binding whose edge matched. */
    internal fun poll() {
        val curr = condition.asBoolean
        val prev = lastState
        lastState = curr
        for (b in bindings) b(prev, curr)
    }
}
