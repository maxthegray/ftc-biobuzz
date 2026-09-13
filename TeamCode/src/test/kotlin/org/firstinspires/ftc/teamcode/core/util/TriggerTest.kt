package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.Gamepad
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/** Trigger edge semantics against Ivy's static scheduler, reset before each test. */
class TriggerTest {

    private lateinit var host: GamepadEx
    private var condition = false

    @Before
    fun setUp() {
        Scheduler.reset()
        host = GamepadEx(Gamepad())
        condition = false
    }

    /** A command that runs until cancelled, so scheduled-ness is observable. */
    private fun endlessCommand(): Command = infinite {}.requiring(Any())

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
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

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
        var ends = 0
        val cmd: Command = Command.build().setDone { done }.setEnd { ends++ }.requiring(Any())
        host.trigger { condition }.whileTrue(cmd)

        condition = true
        poll()
        assertTrue(Scheduler.isScheduled(cmd))

        done = true
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(cmd))

        condition = false
        poll()
        assertFalse(Scheduler.isScheduled(cmd))
        assertEquals(1, ends)
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
        poll()
        assertTrue(Scheduler.isScheduled(cmd))
    }

    @Test
    fun composedTriggersSampleEachConditionOncePerPoll() {
        var leftReads = 0
        var rightReads = 0
        val command = endlessCommand()
        val left = host.trigger { leftReads++; true }
        val right = host.trigger { rightReads++; true }
        (left and right).onTrue(command)

        poll()

        assertEquals(1, leftReads)
        assertEquals(1, rightReads)
        assertTrue(Scheduler.isScheduled(command))
    }

    @Test
    fun aThrowingBindingPropagatesAndIsNotRetriedWhileHeld() {
        var starts = 0
        val bad = Command.build().setStart { starts++; error("start failed") }
        host.trigger { condition }.onTrue(bad)
        condition = true

        assertThrows(IllegalStateException::class.java) { poll() }
        Scheduler.reset()

        // Still true is not a new rising edge.
        poll()
        assertEquals(1, starts)
    }
}
