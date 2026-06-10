package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.After
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Trigger edge semantics against the real Ivy scheduler. The scheduler is a
 * global static, so it is reset around every test.
 */
class TriggerTest {

    private lateinit var host: GamepadEx
    private var condition = false

    @Before
    fun setUp() {
        Scheduler.reset()
        host = GamepadEx(Gamepad())
        condition = false
    }

    @After
    fun tearDown() {
        Scheduler.reset()
    }

    /** A command that runs until cancelled, so scheduled-ness is observable. */
    private fun endlessCommand(): Command = CommandBuilder()
        .setDone { false }
        .requiring(Any())

    private fun poll() = host.update()

    @Test
    fun onTrueSchedulesOnRisingEdgeOnly() {
        val cmd = endlessCommand()
        host.trigger { condition }.onTrue(cmd)

        poll()
        assertFalse(Scheduler.isScheduled(cmd))

        condition = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        // Held true: no re-schedule attempt needed; stays scheduled.
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        // Falling edge does nothing for onTrue.
        condition = false
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun onFalseSchedulesOnFallingEdge() {
        val cmd = endlessCommand()
        host.trigger { condition }.onFalse(cmd)

        condition = true
        poll()
        assertFalse(Scheduler.isScheduled(cmd))

        condition = false
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun whileTrueCancelsOnFallingEdge() {
        val cmd = endlessCommand()
        host.trigger { condition }.whileTrue(cmd)

        condition = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        condition = false
        poll()
        assertFalse(Scheduler.isScheduled(cmd))
    }

    @Test
    fun whileTrueFallingEdgeAfterNaturalEndIsSafe() {
        var done = false
        val cmd: Command = CommandBuilder().setDone { done }.requiring(Any())
        host.trigger { condition }.whileTrue(cmd)

        condition = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        done = true
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(cmd))

        // Falling-edge cancel of an already-finished command must be a no-op.
        condition = false
        poll()
        assertFalse(Scheduler.isScheduled(cmd))
    }

    @Test
    fun toggleOnTrueAlternates() {
        val cmd = endlessCommand()
        host.trigger { condition }.toggleOnTrue(cmd)

        condition = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        condition = false
        poll()
        condition = true
        poll()
        assertFalse(Scheduler.isScheduled(cmd))
    }

    @Test
    fun andCompositionRequiresBoth() {
        var other = false
        val cmd = endlessCommand()
        (host.trigger { condition } and host.trigger { other }).onTrue(cmd)

        condition = true
        poll()
        assertFalse(Scheduler.isScheduled(cmd))

        other = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun notCompositionInverts() {
        val cmd = endlessCommand()
        (!host.trigger { condition }).onTrue(cmd)

        // condition false -> inverted trigger true -> rising edge on first poll.
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun conditionTrueAtFirstPollCountsAsRisingEdge() {
        val cmd = endlessCommand()
        condition = true
        host.trigger { condition }.onTrue(cmd)
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }
}
