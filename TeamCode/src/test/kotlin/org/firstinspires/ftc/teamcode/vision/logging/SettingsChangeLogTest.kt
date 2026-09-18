package org.firstinspires.ftc.teamcode.vision.logging

import org.firstinspires.ftc.teamcode.vision.ball.BallVisionConfig
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SettingsChangeLogTest {

    private val clock = FakeClock()
    private val events = ArrayList<String>()
    private val log = SettingsChangeLog("ballVision", clock, { events += it })

    @Test
    fun firstUpdateWritesEveryValue() {
        log.update(mapOf("a" to "1", "b" to "true"), "detection v1")
        assertEquals(listOf("ballVision (detection v1): a=1 b=true"), events)
    }

    @Test
    fun laterUpdatesWriteOnlyChangedFields() {
        log.update(mapOf("a" to "1", "b" to "2"), "v1")
        clock.advanceMs(2000.0)
        log.update(mapOf("a" to "1", "b" to "3"), "v2")
        assertEquals("ballVision changed (v2): b 2→3", events.last())
    }

    @Test
    fun unchangedValuesWriteNothing() {
        log.update(mapOf("a" to "1"), "v1")
        repeat(5) {
            clock.advanceMs(2000.0)
            log.update(mapOf("a" to "1"), "v1")
        }
        assertEquals(1, events.size)
    }

    @Test
    fun sliderDragIsMergedIntoOneEventPerInterval() {
        log.update(mapOf("a" to "1"), "v1")
        clock.advanceMs(1500.0)
        for (value in 2..20) {
            log.update(mapOf("a" to value.toString()), "v$value")
            clock.advanceMs(20.0)
        }
        assertEquals(2, events.size)
        assertEquals("ballVision changed (v2): a 1→2", events[1])
        assertTrue(log.hasPending)

        clock.advanceMs(1000.0)
        log.update(mapOf("a" to "20"), "v20")
        assertEquals("ballVision changed (v20): a 2→20", events.last())
        assertFalse(log.hasPending)
    }

    @Test
    fun editRevertedInsideTheWindowWritesNothing() {
        log.update(mapOf("a" to "1"), "v1")
        clock.advanceMs(1500.0)
        log.update(mapOf("a" to "2"), "v2")
        log.update(mapOf("a" to "3"), "v3")
        log.update(mapOf("a" to "2"), "v4")
        assertEquals(2, events.size)
        clock.advanceMs(1500.0)
        log.update(mapOf("a" to "2"), "v4")
        assertEquals(2, events.size)
        assertFalse(log.hasPending)
    }

    @Test
    fun valuesOfReadsTunablesInReadableForm() {
        BallVisionConfig.resetDefaults()
        try {
            BallVisionConfig.minCircularity = 0.72
            val values = SettingsChangeLog.valuesOf(BallVisionConfig)
            assertEquals("0.72", values["minCircularity"])
            assertEquals("130", values["channel1Min"])
            assertEquals(BallVisionConfig.compiledDefaults().keys, values.keys)
        } finally {
            BallVisionConfig.resetDefaults()
        }
    }
}
