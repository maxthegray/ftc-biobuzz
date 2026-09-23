package org.firstinspires.ftc.teamcode.core.util

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class ScalarHistoryTest {

    @Test
    fun interpolatesBetweenBracketingSamples() {
        val history = ScalarHistory(8)
        history.add(100, 1.0)
        history.add(200, 3.0)
        assertEquals(2.0, history.lookup(150)!!, 1e-12)
        assertEquals(1.0, history.lookup(100)!!, 0.0)
        assertEquals(3.0, history.lookup(200)!!, 0.0)
    }

    @Test
    fun outsideTheWindowIsNull() {
        val history = ScalarHistory(8)
        assertNull(history.lookup(0))
        history.add(100, 1.0)
        history.add(200, 3.0)
        assertNull(history.lookup(99))
        assertNull(history.lookup(201))
    }

    @Test
    fun anglesAreNotWrapped() {
        val history = ScalarHistory(4)
        history.add(0, 3.0)
        history.add(100, 3.4)
        assertEquals(3.2, history.lookup(50)!!, 1e-12)
    }

    @Test
    fun oldSamplesAreOverwrittenInOrder() {
        val history = ScalarHistory(3)
        for (i in 0 until 5) history.add(i * 10L, i.toDouble())
        assertNull(history.lookup(10))
        assertEquals(2.5, history.lookup(25)!!, 1e-12)
        assertEquals(40L, history.newestTimestamp())
    }

    @Test
    fun aSingleSampleAnswersOnlyItsOwnTime() {
        val history = ScalarHistory(3)
        history.add(10, 0.7)
        assertEquals(0.7, history.lookup(10)!!, 0.0)
        assertNull(history.lookup(11))
        history.clear()
        assertNull(history.newestTimestamp())
    }
}
