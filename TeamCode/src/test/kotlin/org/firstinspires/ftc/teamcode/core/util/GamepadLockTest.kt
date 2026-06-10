package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.After
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class GamepadLockTest {

    private lateinit var host: GamepadEx

    @Before
    fun setUp() {
        Scheduler.reset()
        host = GamepadEx(Gamepad())
    }

    @After
    fun tearDown() {
        Scheduler.reset()
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
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun updateWithoutTriggerPollingSkipsBindings() {
        var condition = false
        val cmd = CommandBuilder().setDone { false }.requiring(Any())
        host.trigger { condition }.onTrue(cmd)

        condition = true
        host.update(pollTriggers = false)
        assertFalse(Scheduler.isScheduled(cmd))

        // Edge detection on raw buttons still works in init mode.
        host.raw.a = true
        host.update(pollTriggers = false)
        assertTrue(host.aPressed)
    }
}
