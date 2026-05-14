package org.firstinspires.ftc.teamcode.core.util

import com.qualcomm.robotcore.hardware.Gamepad
import java.util.function.BooleanSupplier
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.withSign

/**
 * Thin wrapper around [Gamepad] that adds edge detection, deadbands, and
 * exponential sticks. Built for one-line use in op-modes:
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

    /** Exponential curve exponent applied to every stick axis (1.0 = linear). */
    var stickExponent: Double = 2.0

    private val prev = ButtonState()
    private val curr = ButtonState()

    /** Call once per loop before reading any `*Pressed` / `*Released` flags. */
    fun update() {
        prev.copyFrom(curr)
        curr.read(raw)
    }

    val leftStickX: Double get() = curve(raw.left_stick_x.toDouble(), leftStickDeadband)
    val leftStickY: Double get() = curve(-raw.left_stick_y.toDouble(), leftStickDeadband)
    val rightStickX: Double get() = curve(raw.right_stick_x.toDouble(), rightStickDeadband)
    val rightStickY: Double get() = curve(-raw.right_stick_y.toDouble(), rightStickDeadband)

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

    val aReleased: Boolean get() = !curr.a && prev.a
    val bReleased: Boolean get() = !curr.b && prev.b
    val xReleased: Boolean get() = !curr.x && prev.x
    val yReleased: Boolean get() = !curr.y && prev.y

    /** Returns a [BooleanSupplier] that fires once on the leading edge of `a`. */
    fun onAPressed(): BooleanSupplier = BooleanSupplier { aPressed }

    private fun applyDeadband(value: Double, deadband: Double): Double =
        if (abs(value) < deadband) 0.0 else (value - deadband.withSign(value)) / (1.0 - deadband)

    private fun curve(value: Double, deadband: Double): Double {
        val db = applyDeadband(value, deadband)
        if (db == 0.0) return 0.0
        return Math.pow(abs(db), stickExponent) * sign(db)
    }

    private class ButtonState {
        var a = false; var b = false; var x = false; var y = false
        var lb = false; var rb = false
        var up = false; var down = false; var left = false; var right = false
        var start = false; var back = false

        fun read(g: Gamepad) {
            a = g.a; b = g.b; x = g.x; y = g.y
            lb = g.left_bumper; rb = g.right_bumper
            up = g.dpad_up; down = g.dpad_down; left = g.dpad_left; right = g.dpad_right
            start = g.start; back = g.back
        }

        fun copyFrom(o: ButtonState) {
            a = o.a; b = o.b; x = o.x; y = o.y
            lb = o.lb; rb = o.rb
            up = o.up; down = o.down; left = o.left; right = o.right
            start = o.start; back = o.back
        }
    }
}
