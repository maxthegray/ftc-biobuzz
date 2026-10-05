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
 * driver.button(GamepadEx.Button.CROSS).onTrue(intake.grab())
 * val forward = driver.leftStickY
 * ```
 */
class GamepadEx(val raw: Gamepad) {

    var leftStickDeadband: Double = 0.05
    var rightStickDeadband: Double = 0.05
    var triggerDeadband: Double = 0.05

    private val prev = ButtonState()
    private val curr = ButtonState()

    /** Buttons that can back a [Trigger]. L2/R2 are analog: use [trigger] with a threshold. */
    enum class Button {
        CROSS, CIRCLE, SQUARE, TRIANGLE,
        L1, R1,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        OPTIONS, CREATE,
        L3, R3,
        TOUCHPAD,
    }

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
     * Call once per loop before reading any `*Pressed` / `*Released` flags.
     * Also samples every registered [Trigger] (unless [pollTriggers] is
     * false — the init loop disables it so bindings can't start commands
     * before the match), so trigger-bound commands are scheduled or
     * cancelled here, outside `Scheduler.execute()`. A throwing condition or
     * binding propagates to [Robot],
     * which aborts all commands and halts the subsystems.
     */
    fun update(pollTriggers: Boolean = true) {
        prev.copyFrom(curr)
        curr.read(raw)
        if (pollTriggers) {
            for (t in triggers) t.poll()
        }
    }

    val leftStickX: Double get() = applyDeadband(raw.left_stick_x.toDouble(), leftStickDeadband)
    val leftStickY: Double get() = applyDeadband(-raw.left_stick_y.toDouble(), leftStickDeadband)
    val rightStickX: Double get() = applyDeadband(raw.right_stick_x.toDouble(), rightStickDeadband)
    val rightStickY: Double get() = applyDeadband(-raw.right_stick_y.toDouble(), rightStickDeadband)

    val l2: Double get() = applyDeadband(raw.left_trigger.toDouble(), triggerDeadband)
    val r2: Double get() = applyDeadband(raw.right_trigger.toDouble(), triggerDeadband)

    val cross: Boolean get() = curr.cross
    val circle: Boolean get() = curr.circle
    val square: Boolean get() = curr.square
    val triangle: Boolean get() = curr.triangle
    val l1: Boolean get() = curr.l1
    val r1: Boolean get() = curr.r1
    val dpadUp: Boolean get() = curr.up
    val dpadDown: Boolean get() = curr.down
    val dpadLeft: Boolean get() = curr.left
    val dpadRight: Boolean get() = curr.right
    val options: Boolean get() = curr.options
    val create: Boolean get() = curr.create
    val l3: Boolean get() = curr.l3
    val r3: Boolean get() = curr.r3
    val touchpad: Boolean get() = curr.touchpad

    val crossPressed: Boolean get() = curr.cross && !prev.cross
    val circlePressed: Boolean get() = curr.circle && !prev.circle
    val squarePressed: Boolean get() = curr.square && !prev.square
    val trianglePressed: Boolean get() = curr.triangle && !prev.triangle
    val l1Pressed: Boolean get() = curr.l1 && !prev.l1
    val r1Pressed: Boolean get() = curr.r1 && !prev.r1
    val dpadUpPressed: Boolean get() = curr.up && !prev.up
    val dpadDownPressed: Boolean get() = curr.down && !prev.down
    val dpadLeftPressed: Boolean get() = curr.left && !prev.left
    val dpadRightPressed: Boolean get() = curr.right && !prev.right
    val l3Pressed: Boolean get() = curr.l3 && !prev.l3
    val r3Pressed: Boolean get() = curr.r3 && !prev.r3
    val touchpadPressed: Boolean get() = curr.touchpad && !prev.touchpad

    val crossReleased: Boolean get() = !curr.cross && prev.cross
    val circleReleased: Boolean get() = !curr.circle && prev.circle
    val squareReleased: Boolean get() = !curr.square && prev.square
    val triangleReleased: Boolean get() = !curr.triangle && prev.triangle

    fun rumble(ms: Int) {
        raw.rumble(ms)
    }

    fun rumbleBlips(count: Int) {
        raw.rumbleBlips(count)
    }

    /**
     * A [Trigger] backed by [button]. Cached — repeated calls for the same
     * button return the same Trigger, so bindings accumulate on one instance.
     */
    fun button(button: Button): Trigger =
        buttonTriggers.getOrPut(button) { trigger { stateOf(button) } }

    /**
     * A [Trigger] backed by an arbitrary condition — an analog stick or trigger
     * past a threshold, a sensor reading, anything. Sampled once per loop in
     * [update]; each call returns a fresh Trigger.
     */
    fun trigger(condition: BooleanSupplier): Trigger {
        requireBindingsUnlocked()
        return Trigger(this, condition).also { triggers += it }
    }

    private fun stateOf(button: Button): Boolean = when (button) {
        Button.CROSS -> curr.cross
        Button.CIRCLE -> curr.circle
        Button.SQUARE -> curr.square
        Button.TRIANGLE -> curr.triangle
        Button.L1 -> curr.l1
        Button.R1 -> curr.r1
        Button.DPAD_UP -> curr.up
        Button.DPAD_DOWN -> curr.down
        Button.DPAD_LEFT -> curr.left
        Button.DPAD_RIGHT -> curr.right
        Button.OPTIONS -> curr.options
        Button.CREATE -> curr.create
        Button.L3 -> curr.l3
        Button.R3 -> curr.r3
        Button.TOUCHPAD -> curr.touchpad
    }

    private class ButtonState {
        var cross = false; var circle = false; var square = false; var triangle = false
        var l1 = false; var r1 = false
        var up = false; var down = false; var left = false; var right = false
        var options = false; var create = false
        var l3 = false; var r3 = false
        var touchpad = false

        fun read(g: Gamepad) {
            cross = g.cross; circle = g.circle; square = g.square; triangle = g.triangle
            l1 = g.left_bumper; r1 = g.right_bumper
            up = g.dpad_up; down = g.dpad_down; left = g.dpad_left; right = g.dpad_right
            options = g.options; create = g.share
            l3 = g.left_stick_button; r3 = g.right_stick_button
            touchpad = g.touchpad
        }

        fun copyFrom(o: ButtonState) {
            cross = o.cross; circle = o.circle; square = o.square; triangle = o.triangle
            l1 = o.l1; r1 = o.r1
            up = o.up; down = o.down; left = o.left; right = o.right
            options = o.options; create = o.create
            l3 = o.l3; r3 = o.r3
            touchpad = o.touchpad
        }
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
