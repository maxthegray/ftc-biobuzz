package org.firstinspires.ftc.teamcode.core.util

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class MonotonicWaitTest {
    private val clock = FakeClock()

    @Before
    fun resetScheduler() = Scheduler.reset()

    @Test
    fun endsWhenTheMonotonicClockHasAdvancedAndNotBefore() {
        val wait = monotonicWaitMs(100.0, clock)
        Scheduler.schedule(wait)
        clock.advanceMs(99.9)
        Scheduler.execute()
        assertTrue(Scheduler.isScheduled(wait))
        clock.advanceMs(0.1)
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(wait))
    }

    @Test
    fun ignoresWallClockTimeEntirely() {
        val wait = monotonicWaitMs(1.0, clock)
        Scheduler.schedule(wait)
        // Real time passes (as if the wall clock jumped): nothing happens.
        val wallStart = System.currentTimeMillis()
        while (System.currentTimeMillis() - wallStart < 5) Thread.onSpinWait()
        Scheduler.execute()
        assertTrue(Scheduler.isScheduled(wait))
        // Monotonic time passes with no real time: it ends.
        clock.advanceMs(1.0)
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(wait))
    }

    @Test
    fun restartsAtEveryStartAndCancelsCleanly() {
        val wait = monotonicWaitMs(50.0, clock)
        Scheduler.schedule(wait)
        clock.advanceMs(40.0)
        Scheduler.cancel(wait)
        assertFalse(Scheduler.isScheduled(wait))

        clock.advanceMs(100.0)
        Scheduler.schedule(wait)
        Scheduler.execute()
        assertTrue("timer restarted at the second start", Scheduler.isScheduled(wait))
        clock.advanceMs(50.0)
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(wait))
    }

    @Test
    fun composesAsATimeoutAndInSequence() {
        val log = mutableListOf<String>()
        var stepEnded: String? = null
        val routine = sequential(
            race(infinite {}.setEnd { stepEnded = it.name }, monotonicWaitMs(30.0, clock)),
            instant { log += "after timeout" },
            monotonicWaitMs(20.0, clock),
            instant { log += "done" },
        )
        Scheduler.schedule(routine)
        repeat(3) {
            clock.advanceMs(10.0)
            Scheduler.execute()
        }
        Scheduler.execute()
        assertEquals("INTERRUPTED", stepEnded)
        assertEquals(listOf("after timeout"), log)
        // The instant ends and the second wait starts on this tick.
        Scheduler.execute()
        clock.advanceMs(19.9)
        Scheduler.execute()
        assertEquals(listOf("after timeout"), log)
        clock.advanceMs(0.1)
        Scheduler.execute()
        Scheduler.execute()
        assertEquals(listOf("after timeout", "done"), log)
        assertFalse(Scheduler.isScheduled(routine))
    }

    @Test
    fun zeroEndsOnTheFirstCheckAndInvalidDurationsAreRejected() {
        val wait = monotonicWaitMs(0.0, clock)
        Scheduler.schedule(wait)
        Scheduler.execute()
        assertFalse(Scheduler.isScheduled(wait))
        assertThrows(IllegalArgumentException::class.java) { monotonicWaitMs(-1.0, clock) }
        assertThrows(IllegalArgumentException::class.java) { monotonicWaitMs(Double.NaN, clock) }
    }
}
