package org.firstinspires.ftc.teamcode.core.util

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class MatchTimerTest {

    @Test
    fun elapsedAndRemainingUseInjectedClock() {
        val clock = FakeClock(start = 0L)
        val timer = MatchTimer(clock)

        timer.start()
        clock.advanceMs(12_500.0)

        assertEquals(12.5, timer.elapsedSec, 1e-9)
        assertEquals(107.5, timer.remainingSec(120.0), 1e-9)
    }

    @Test
    fun endgameStartsAtThreshold() {
        val clock = FakeClock(start = 0L)
        val timer = MatchTimer(clock)

        timer.start()
        clock.advanceMs(89_999.0)
        assertFalse(timer.inEndgame())

        clock.advanceMs(1.0)
        assertTrue(timer.inEndgame())
    }
}
