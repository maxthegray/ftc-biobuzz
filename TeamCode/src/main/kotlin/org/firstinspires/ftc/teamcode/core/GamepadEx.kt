package org.firstinspires.ftc.teamcode.core

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.Gamepad
import java.util.function.BooleanSupplier
import kotlin.math.abs
import kotlin.math.withSign

/**
 * Thin wrapper around a PS5 controller's [Gamepad] that adds edge detection,
 * deadbanded axes (sticks read +up/+right), and [Trigger] bindings that
 * schedule Ivy commands. Ivy itself has no gamepad bindings; this is the
 * input helper. Names follow the PS5 labels; the SDK calls Create `share`.
 *
 * ```kotlin
 * driver.button(Button.CROSS).onTrue(intake.grab())   // bind in configure()
 * if (driver.pressed(Button.DPAD_UP)) delay++         // edge, e.g. in onInitLoop()
 * val forward = driver.leftStickY
 * ```
 */
class GamepadEx(val raw: Gamepad) {

    /**
     * Digital buttons. L2/R2 are analog: use [trigger] with a threshold. The
     * order is the bit order of the flight log's `gamepadN/buttons` channel.
     */
    enum class Button {
        CROSS, CIRCLE, SQUARE, TRIANGLE,
        L1, R1,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        OPTIONS, CREATE,
        L3, R3,
        TOUCHPAD,
    }

    private val prev = BooleanArray(Button.entries.size)
    private val curr = BooleanArray(Button.entries.size)

    private val triggers = mutableListOf<Trigger>()
    private val buttonTriggers = HashMap<Button, Trigger>()

    /**
     * True once the op-mode has started. After this, creating triggers or
     * adding bindings throws — bindings created per-tick from `onLoop()`
     * accumulate forever and re-schedule commands unboundedly, so the
     * framework forces them into `configure()`.
     */
    var bindingsLocked: Boolean = false
        private set

    fun lockBindings() {
        bindingsLocked = true
        // Triggers are not polled during init, so lastState is stale false.
        // Prime from the current condition so a button held across start
        // doesn't fire its binding as a phantom rising edge on the first poll.
        for (t in triggers) t.prime()
    }

    internal fun requireBindingsUnlocked() {
        check(!bindingsLocked) {
            "Gamepad triggers/bindings are locked once the op-mode starts — wire them in configure()."
        }
    }

    /**
     * Call once per loop before reading [held] / [pressed] / [released].
     * Also samples every registered [Trigger] (unless [pollTriggers] is
     * false — the init loop disables it so bindings can't start commands
     * before the match), so trigger-bound commands are scheduled or
     * cancelled here, outside `Scheduler.execute()`. A throwing condition or
     * binding propagates to [Robot],
     * which aborts all commands and halts the subsystems.
     */
    fun update(pollTriggers: Boolean = true) {
        curr.copyInto(prev)
        for (b in Button.entries) curr[b.ordinal] = read(b)
        if (pollTriggers) {
            for (t in triggers) t.poll()
        }
    }

    val leftStickX: Double get() = applyDeadband(raw.left_stick_x.toDouble(), DEADBAND)
    val leftStickY: Double get() = applyDeadband(-raw.left_stick_y.toDouble(), DEADBAND)
    val rightStickX: Double get() = applyDeadband(raw.right_stick_x.toDouble(), DEADBAND)
    val rightStickY: Double get() = applyDeadband(-raw.right_stick_y.toDouble(), DEADBAND)

    val l2: Double get() = applyDeadband(raw.left_trigger.toDouble(), DEADBAND)
    val r2: Double get() = applyDeadband(raw.right_trigger.toDouble(), DEADBAND)

    /** [button] is down this loop. */
    fun held(button: Button): Boolean = curr[button.ordinal]

    /** [button] went down since the last [update]. */
    fun pressed(button: Button): Boolean = curr[button.ordinal] && !prev[button.ordinal]

    /** [button] came up since the last [update]. */
    fun released(button: Button): Boolean = !curr[button.ordinal] && prev[button.ordinal]

    fun rumble(ms: Int) {
        raw.rumble(ms)
    }

    /**
     * A [Trigger] backed by [button]. Cached — repeated calls for the same
     * button return the same Trigger, so bindings accumulate on one instance.
     */
    fun button(button: Button): Trigger =
        buttonTriggers.getOrPut(button) { trigger { held(button) } }

    /**
     * A [Trigger] backed by an arbitrary condition — an analog stick or trigger
     * past a threshold, a sensor reading, anything. Sampled once per loop in
     * [update]; each call returns a fresh Trigger.
     */
    fun trigger(condition: BooleanSupplier): Trigger {
        requireBindingsUnlocked()
        return Trigger(this, condition).also { triggers += it }
    }

    private fun read(button: Button): Boolean = when (button) {
        Button.CROSS -> raw.cross
        Button.CIRCLE -> raw.circle
        Button.SQUARE -> raw.square
        Button.TRIANGLE -> raw.triangle
        Button.L1 -> raw.left_bumper
        Button.R1 -> raw.right_bumper
        Button.DPAD_UP -> raw.dpad_up
        Button.DPAD_DOWN -> raw.dpad_down
        Button.DPAD_LEFT -> raw.dpad_left
        Button.DPAD_RIGHT -> raw.dpad_right
        Button.OPTIONS -> raw.options
        Button.CREATE -> raw.share
        Button.L3 -> raw.left_stick_button
        Button.R3 -> raw.right_stick_button
        Button.TOUCHPAD -> raw.touchpad
    }

    private companion object {
        /** Stick and L2/R2 travel that reads as zero. */
        const val DEADBAND = 0.05
    }
}

/**
 * Scaled deadband: inputs inside ±[deadband] read 0, and the remaining range
 * is re-normalised so output still spans the full ±1 — no dead jump at the
 * deadband edge.
 */
internal fun applyDeadband(value: Double, deadband: Double): Double =
    if (abs(value) < deadband) 0.0 else (value - deadband.withSign(value)) / (1.0 - deadband)

/**
 * A boolean condition — a gamepad button, an analog trigger past a threshold,
 * a sensor state — bound to Ivy [Command]s. Wire bindings once in
 * `configure()`:
 *
 * ```kotlin
 * driver.button(GamepadEx.Button.CROSS).onTrue(intake.grab())
 * driver.button(GamepadEx.Button.L1).whileTrue(intake.eject())
 * driver.trigger { driver.r2 > 0.5 }.whileTrue(drive.slowMode())
 * (driver.button(GamepadEx.Button.CREATE) and driver.button(GamepadEx.Button.TRIANGLE))
 *     .onTrue(resetHeading())
 * ```
 *
 * Create one through [GamepadEx.button] / [GamepadEx.trigger], or by composing
 * existing triggers with [and] / [or]. The owning [GamepadEx] samples
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

    /** Schedule [command] on the rising edge and cancel it on the falling edge. */
    fun whileTrue(command: Command): Trigger = bind { prev, curr ->
        if (!prev && curr) {
            Scheduler.schedule(command)
        } else if (prev && !curr) {
            Scheduler.cancel(command)
        }
    }

    /** Active only while both this and [other] are active. */
    infix fun and(other: Trigger): Trigger = host.trigger { read() && other.read() }

    /** Active while either this or [other] is active. */
    infix fun or(other: Trigger): Trigger = host.trigger { read() || other.read() }

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
