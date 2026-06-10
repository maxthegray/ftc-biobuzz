package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import java.util.function.BooleanSupplier

/**
 * A boolean condition — a gamepad button, an analog trigger past a threshold,
 * a sensor state — bound to Ivy [Command]s.
 *
 * Triggers replace the imperative `if (driver.aPressed) Scheduler.schedule(cmd)`
 * pattern with declarative bindings wired once at configure-time:
 *
 * ```kotlin
 * override fun configure() {
 *     val intake = robot.register(IntakeSubsystem(...))
 *     driver.button(GamepadEx.Button.A).onTrue(intake.grab())
 *     driver.button(GamepadEx.Button.LEFT_BUMPER).whileTrue(intake.eject())
 *     driver.trigger { driver.rightTrigger > 0.5 }.whileTrue(drive.slowMode())
 *     (driver.button(GamepadEx.Button.START) and driver.button(GamepadEx.Button.A))
 *         .onTrue(resetHeading())
 * }
 * ```
 *
 * Create one through [GamepadEx.button] / [GamepadEx.trigger], or by composing
 * existing triggers with [and] / [or] / [not] — never construct directly. The
 * owning [GamepadEx] samples every trigger once per loop from
 * [GamepadEx.update], which runs before the command scheduler ticks.
 *
 * Edge semantics: the condition is sampled once per loop. A "rising edge" is a
 * loop where it read false last tick and true this tick; a "falling edge" is
 * the reverse.
 *
 * Composition caveat: compose triggers from the same [GamepadEx] host. A
 * cross-gamepad composition is polled by the left-hand trigger's host, so the
 * other gamepad may still be on its previous loop sample.
 */
class Trigger internal constructor(
    private val host: GamepadEx,
    private val condition: BooleanSupplier,
) {
    private var lastState = false
    private val bindings = mutableListOf<(prev: Boolean, curr: Boolean) -> Unit>()

    /** Schedule [command] once on each rising edge (skipped if it is already scheduled). */
    fun onTrue(command: Command): Trigger {
        bindings += { prev, curr ->
            if (!prev && curr && !Scheduler.isScheduled(command)) Scheduler.schedule(command)
        }
        return this
    }

    /** Schedule [command] once on each falling edge (skipped if it is already scheduled). */
    fun onFalse(command: Command): Trigger {
        bindings += { prev, curr ->
            if (prev && !curr && !Scheduler.isScheduled(command)) Scheduler.schedule(command)
        }
        return this
    }

    /**
     * Schedule [command] on the rising edge and cancel it on the falling edge.
     * If the command ends on its own first, the falling-edge cancel is a no-op
     * — Ivy's [Scheduler.cancel] is safe on an unscheduled command.
     */
    fun whileTrue(command: Command): Trigger {
        bindings += { prev, curr ->
            if (!prev && curr) {
                if (!Scheduler.isScheduled(command)) Scheduler.schedule(command)
            } else if (prev && !curr) {
                Scheduler.cancel(command)
            }
        }
        return this
    }

    /**
     * On each rising edge, toggle [command]: schedule it if it is not running,
     * cancel it if it is.
     */
    fun toggleOnTrue(command: Command): Trigger {
        bindings += { prev, curr ->
            if (!prev && curr) {
                if (Scheduler.isScheduled(command)) Scheduler.cancel(command)
                else Scheduler.schedule(command)
            }
        }
        return this
    }

    /** Active only while both this and [other] are active. Prefer same-host composition. */
    infix fun and(other: Trigger): Trigger = host.trigger { read() && other.read() }

    /** Active while either this or [other] is active. Prefer same-host composition. */
    infix fun or(other: Trigger): Trigger = host.trigger { read() || other.read() }

    /** Active exactly when this trigger is not. Also usable as `!trigger`. */
    operator fun not(): Trigger = host.trigger { !read() }

    internal fun read(): Boolean = condition.asBoolean

    /** Sample the condition and fire any binding whose edge matched. Called once per loop by [host]. */
    internal fun poll() {
        val curr = condition.asBoolean
        for (b in bindings) b(lastState, curr)
        lastState = curr
    }
}
