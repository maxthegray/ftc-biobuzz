package org.firstinspires.ftc.teamcode.core.util

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.command.Scheduler
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Trigger edge semantics against a real scheduler instance — fresh per test,
 * nothing global to reset.
 */
class TriggerTest {

    private lateinit var scheduler: Scheduler
    private lateinit var host: GamepadEx
    private var condition = false

    @Before
    fun setUp() {
        scheduler = Scheduler()
        host = GamepadEx(Gamepad(), scheduler)
        condition = false
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
        assertFalse(scheduler.isScheduled(cmd))

        condition = true
        poll()
        assertTrue(scheduler.isScheduled(cmd))

        // Held true: no re-schedule attempt needed; stays scheduled.
        poll()
        assertTrue(scheduler.isScheduled(cmd))

        // Falling edge does nothing for onTrue.
        condition = false
        poll()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun onFalseSchedulesOnFallingEdge() {
        val cmd = endlessCommand()
        host.trigger { condition }.onFalse(cmd)

        condition = true
        poll()
        assertFalse(scheduler.isScheduled(cmd))

        condition = false
        poll()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun whileTrueCancelsOnFallingEdge() {
        val cmd = endlessCommand()
        host.trigger { condition }.whileTrue(cmd)

        condition = true
        poll()
        assertTrue(scheduler.isScheduled(cmd))

        condition = false
        poll()
        assertFalse(scheduler.isScheduled(cmd))
    }

    @Test
    fun whileTrueFallingEdgeAfterNaturalEndIsSafe() {
        var done = false
        val cmd: Command = CommandBuilder().setDone { done }.requiring(Any())
        host.trigger { condition }.whileTrue(cmd)

        condition = true
        poll()
        assertTrue(scheduler.isScheduled(cmd))

        done = true
        scheduler.execute()
        assertFalse(scheduler.isScheduled(cmd))

        // Falling-edge cancel of an already-finished command must be a no-op.
        condition = false
        poll()
        assertFalse(scheduler.isScheduled(cmd))
    }

    @Test
    fun toggleOnTrueAlternates() {
        val cmd = endlessCommand()
        host.trigger { condition }.toggleOnTrue(cmd)

        condition = true
        poll()
        assertTrue(scheduler.isScheduled(cmd))

        condition = false
        poll()
        condition = true
        poll()
        assertFalse(scheduler.isScheduled(cmd))
    }

    @Test
    fun andCompositionRequiresBoth() {
        var other = false
        val cmd = endlessCommand()
        (host.trigger { condition } and host.trigger { other }).onTrue(cmd)

        condition = true
        poll()
        assertFalse(scheduler.isScheduled(cmd))

        other = true
        poll()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun notCompositionInverts() {
        val cmd = endlessCommand()
        (!host.trigger { condition }).onTrue(cmd)

        // condition false -> inverted trigger true -> rising edge on first poll.
        poll()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun conditionTrueAtFirstPollCountsAsRisingEdge() {
        val cmd = endlessCommand()
        condition = true
        host.trigger { condition }.onTrue(cmd)
        poll()
        assertTrue(scheduler.isScheduled(cmd))
    }

    @Test
    fun failingTriggerIsQuarantinedWithoutStarvingLaterTriggers() {
        var failures = 0
        var badReads = 0
        val isolatedHost = GamepadEx(Gamepad(), scheduler) { failures++ }
        val good = endlessCommand()
        isolatedHost.trigger {
            badReads++
            error("sensor failed")
        }.onTrue(endlessCommand())
        isolatedHost.trigger { condition }.onTrue(good)

        condition = true
        isolatedHost.update()

        assertEquals(1, failures)
        assertEquals(1, badReads)
        assertTrue(scheduler.isScheduled(good))

        isolatedHost.update()
        assertEquals(1, failures)
        assertEquals(1, badReads)
    }

    @Test
    fun quarantiningAnActiveWhileTrueTriggerCancelsItsCommand() {
        var shouldThrow = false
        var failures = 0
        val isolatedHost = GamepadEx(Gamepad(), scheduler) { failures++ }
        val command = endlessCommand()
        isolatedHost.trigger {
            if (shouldThrow) error("sensor failed")
            true
        }.whileTrue(command)

        isolatedHost.update()
        assertTrue(scheduler.isScheduled(command))

        shouldThrow = true
        isolatedHost.update()

        assertEquals(1, failures)
        assertFalse(scheduler.isScheduled(command))
    }

    @Test
    fun composedTriggersSampleEachConditionOncePerPoll() {
        var leftReads = 0
        var rightReads = 0
        val command = endlessCommand()
        val left = host.trigger {
            leftReads++
            true
        }
        val right = host.trigger {
            rightReads++
            true
        }
        (left and right).onTrue(command)

        poll()

        assertEquals(1, leftReads)
        assertEquals(1, rightReads)
        assertTrue(scheduler.isScheduled(command))
    }

    @Test
    fun pollAdvancesSampledStateWhenABindingThrows() {
        val bad = CommandBuilder().setStart { error("start failed") }
        val trigger = host.trigger { condition }.onTrue(bad)
        condition = true

        assertThrows(IllegalStateException::class.java) { trigger.poll() }

        // Still true is no longer a rising edge, so the bad binding is not retried.
        trigger.poll()
    }
}
