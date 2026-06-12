package org.firstinspires.ftc.teamcode.core.util

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.command.Scheduler
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class GamepadLockTest {

    private lateinit var scheduler: Scheduler
    private lateinit var host: GamepadEx

    @Before
    fun setUp() {
        scheduler = Scheduler()
        host = GamepadEx(Gamepad(), scheduler)
    }

    @Test(expected = IllegalStateException::class)
    fun creatingTriggerAfterLockThrows() {
        host.lockBindings()
        host.trigger { true }
    }

    @Test(expected = IllegalStateException::class)
    fun creatingButtonTriggerAfterLockThrows() {
        host.lockBindings()
        host.button(GamepadEx.Button.A)
    }

    @Test(expected = IllegalStateException::class)
    fun bindingOnExistingTriggerAfterLockThrows() {
        val t = host.button(GamepadEx.Button.A)
        host.lockBindings()
        t.onTrue(CommandBuilder())
    }

    @Test(expected = IllegalStateException::class)
    fun composingAfterLockThrows() {
        val a = host.button(GamepadEx.Button.A)
        val b = host.button(GamepadEx.Button.B)
        host.lockBindings()
        a and b
    }

    @Test
    fun preLockBindingsKeepWorkingAfterLock() {
        var condition = false
        val cmd = CommandBuilder().setDone { false }.requiring(Any())
        host.trigger { condition }.onTrue(cmd)
        host.lockBindings()

        condition = true
        host.update()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun buttonHeldThroughInitDoesNotFireAtStart() {
        val cmd = CommandBuilder().setDone { false }.requiring(Any())
        host.button(GamepadEx.Button.A).onTrue(cmd)

        // Init loop: button pressed and held, triggers not polled.
        host.raw.a = true
        host.update(pollTriggers = false)

        // Start: lockBindings primes edge state; still held -> no rising edge.
        host.lockBindings()
        host.update()
        assertFalse(scheduler.isScheduled(cmd))

        // Release and re-press: a real rising edge fires normally.
        host.raw.a = false
        host.update()
        host.raw.a = true
        host.update()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun buttonNotHeldAtStartStillFiresOnFirstPress() {
        val cmd = CommandBuilder().setDone { false }.requiring(Any())
        host.button(GamepadEx.Button.A).onTrue(cmd)

        host.update(pollTriggers = false)
        host.lockBindings()

        host.raw.a = true
        host.update()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun updateWithoutTriggerPollingSkipsBindings() {
        var condition = false
        val cmd = CommandBuilder().setDone { false }.requiring(Any())
        host.trigger { condition }.onTrue(cmd)

        condition = true
        host.update(pollTriggers = false)
        assertFalse(scheduler.isScheduled(cmd))

        // Edge detection on raw buttons still works in init mode.
        host.raw.a = true
        host.update(pollTriggers = false)
        assertTrue(host.aPressed)
    }
}
