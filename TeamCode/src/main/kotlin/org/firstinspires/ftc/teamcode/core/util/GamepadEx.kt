package org.firstinspires.ftc.teamcode.core.util

import com.qualcomm.robotcore.hardware.Gamepad
import java.util.function.BooleanSupplier
import kotlin.math.abs
import kotlin.math.withSign

/**
 * Thin wrapper around [Gamepad] that adds edge detection and deadbanded axes.
 * Built for one-line use in op-modes:
 *
 * ```kotlin
 * val driver = GamepadEx(gamepad1)
 * // inside loop:
 * driver.update()
 * if (driver.aPressed) intake.togglePressed()
 * drive.arcade(driver.leftStickY, driver.leftStickX, driver.rightStickX)
 * ```
 */
class GamepadEx(val raw: Gamepad) {

    var leftStickDeadband: Double = 0.05
    var rightStickDeadband: Double = 0.05
    var triggerDeadband: Double = 0.05

    private val prev = ButtonState()
    private val curr = ButtonState()

    /** Buttons that can back a [Trigger]. Analog inputs go through [trigger] with a threshold. */
    enum class Button {
        A, B, X, Y,
        LEFT_BUMPER, RIGHT_BUMPER,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        START, BACK,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
    }

    private val triggers = mutableListOf<Trigger>()
    private val buttonTriggers = HashMap<Button, Trigger>()
    private var bindingsFrozen = false

    /**
     * Called by `OpModeBase` once the main loop starts. Every [trigger] /
     * [button] / composition call registers a binding that is polled
     * forever — creating one per loop iteration leaks and double-fires, so
     * after this point trigger creation throws instead.
     */
    fun freezeBindings() {
        bindingsFrozen = true
    }

    /**
     * Call once per loop before reading any `*Pressed` / `*Released` flags.
     * Also samples every registered [Trigger], so trigger-bound commands are
     * scheduled or cancelled here — before the command scheduler ticks.
     *
     * Pass `pollTriggers = false` to refresh button state without firing
     * trigger bindings — used during the init loop, where buttons should be
     * readable (alliance/start-position selection) but bindings must not
     * queue Ivy commands before the op-mode starts.
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

    val leftTrigger: Double get() = applyDeadband(raw.left_trigger.toDouble(), triggerDeadband)
    val rightTrigger: Double get() = applyDeadband(raw.right_trigger.toDouble(), triggerDeadband)

    val a: Boolean get() = curr.a
    val b: Boolean get() = curr.b
    val x: Boolean get() = curr.x
    val y: Boolean get() = curr.y
    val leftBumper: Boolean get() = curr.lb
    val rightBumper: Boolean get() = curr.rb
    val dpadUp: Boolean get() = curr.up
    val dpadDown: Boolean get() = curr.down
    val dpadLeft: Boolean get() = curr.left
    val dpadRight: Boolean get() = curr.right
    val start: Boolean get() = curr.start
    val back: Boolean get() = curr.back
    val leftStickButton: Boolean get() = curr.leftStick
    val rightStickButton: Boolean get() = curr.rightStick

    val aPressed: Boolean get() = curr.a && !prev.a
    val bPressed: Boolean get() = curr.b && !prev.b
    val xPressed: Boolean get() = curr.x && !prev.x
    val yPressed: Boolean get() = curr.y && !prev.y
    val leftBumperPressed: Boolean get() = curr.lb && !prev.lb
    val rightBumperPressed: Boolean get() = curr.rb && !prev.rb
    val dpadUpPressed: Boolean get() = curr.up && !prev.up
    val dpadDownPressed: Boolean get() = curr.down && !prev.down
    val dpadLeftPressed: Boolean get() = curr.left && !prev.left
    val dpadRightPressed: Boolean get() = curr.right && !prev.right
    val leftStickButtonPressed: Boolean get() = curr.leftStick && !prev.leftStick
    val rightStickButtonPressed: Boolean get() = curr.rightStick && !prev.rightStick

    val aReleased: Boolean get() = !curr.a && prev.a
    val bReleased: Boolean get() = !curr.b && prev.b
    val xReleased: Boolean get() = !curr.x && prev.x
    val yReleased: Boolean get() = !curr.y && prev.y

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
     *
     * Note: [update] runs *before* the Lynx bulk caches are cleared for the
     * tick, so a condition that reads a motor/encoder/sensor sees last
     * tick's value. That one-tick latency is fine for command scheduling;
     * don't use trigger conditions for anything that needs same-tick data.
     */
    fun trigger(condition: BooleanSupplier): Trigger {
        check(!bindingsFrozen) {
            "Triggers must be created in configure()/onStart, not mid-loop — each call registers a new binding that is polled forever."
        }
        return Trigger(this, condition).also { triggers += it }
    }

    private fun stateOf(button: Button): Boolean = when (button) {
        Button.A -> curr.a
        Button.B -> curr.b
        Button.X -> curr.x
        Button.Y -> curr.y
        Button.LEFT_BUMPER -> curr.lb
        Button.RIGHT_BUMPER -> curr.rb
        Button.DPAD_UP -> curr.up
        Button.DPAD_DOWN -> curr.down
        Button.DPAD_LEFT -> curr.left
        Button.DPAD_RIGHT -> curr.right
        Button.START -> curr.start
        Button.BACK -> curr.back
        Button.LEFT_STICK_BUTTON -> curr.leftStick
        Button.RIGHT_STICK_BUTTON -> curr.rightStick
    }

    private fun applyDeadband(value: Double, deadband: Double): Double =
        if (abs(value) < deadband) 0.0 else (value - deadband.withSign(value)) / (1.0 - deadband)

    private class ButtonState {
        var a = false; var b = false; var x = false; var y = false
        var lb = false; var rb = false
        var up = false; var down = false; var left = false; var right = false
        var start = false; var back = false
        var leftStick = false; var rightStick = false

        fun read(g: Gamepad) {
            a = g.a; b = g.b; x = g.x; y = g.y
            lb = g.left_bumper; rb = g.right_bumper
            up = g.dpad_up; down = g.dpad_down; left = g.dpad_left; right = g.dpad_right
            start = g.start; back = g.back
            leftStick = g.left_stick_button; rightStick = g.right_stick_button
        }

        fun copyFrom(o: ButtonState) {
            a = o.a; b = o.b; x = o.x; y = o.y
            lb = o.lb; rb = o.rb
            up = o.up; down = o.down; left = o.left; right = o.right
            start = o.start; back = o.back
            leftStick = o.leftStick; rightStick = o.rightStick
        }
    }
}
