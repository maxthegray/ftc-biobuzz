package org.firstinspires.ftc.teamcode.vision.hive

import org.firstinspires.ftc.teamcode.vision.apriltags.BiobuzzAprilTags.Alliance
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HiveStateTest {

    private val ms = 1_000_000L

    @Test
    fun matchSetupPriorIsAssumedUntilAFrameConfirmsIt() {
        val hive = HiveStateEstimator(Alliance.RED, HiveState.matchSetup(Alliance.RED))
        assertEquals(HiveState.AUDIENCE_RAISED, hive.state)
        assertTrue(hive.assumed)

        assertNull(hive.observe(HiveState.AUDIENCE_RAISED, 10 * ms, 3))
        assertFalse(hive.assumed)
        assertEquals(10 * ms, hive.lastObservedNanos)
        assertEquals(0, hive.tipCount)
    }

    @Test
    fun blueStartsWithItsFarCellRaised() {
        assertEquals(HiveState.FAR_RAISED, HiveState.matchSetup(Alliance.BLUE))
    }

    @Test
    fun oneContradictingFrameCannotFakeATip() {
        val hive = HiveStateEstimator(Alliance.RED, HiveState.AUDIENCE_RAISED)
        hive.observe(HiveState.AUDIENCE_RAISED, 0, 3)
        assertNull(hive.observe(HiveState.FAR_RAISED, 10 * ms, 3))
        assertNull(hive.observe(HiveState.AUDIENCE_RAISED, 20 * ms, 3))
        assertNull(hive.observe(HiveState.FAR_RAISED, 30 * ms, 3))
        assertNull(hive.observe(HiveState.FAR_RAISED, 40 * ms, 3))
        assertEquals(HiveState.AUDIENCE_RAISED, hive.state)
        assertEquals(0, hive.tipCount)
    }

    @Test
    fun consecutiveFramesConfirmATip() {
        val hive = HiveStateEstimator(Alliance.BLUE, HiveState.FAR_RAISED)
        hive.observe(HiveState.FAR_RAISED, 0, 3)
        hive.observe(HiveState.AUDIENCE_RAISED, 10 * ms, 3)
        hive.observe(HiveState.AUDIENCE_RAISED, 20 * ms, 3)
        val tip = hive.observe(HiveState.AUDIENCE_RAISED, 30 * ms, 3)

        assertNotNull(tip)
        assertTrue(tip!!.isTip)
        assertEquals(HiveState.AUDIENCE_RAISED, hive.state)
        assertEquals(1, hive.tipCount)
        assertEquals(30 * ms, hive.lastTipNanos)
        assertEquals("HIVE TIP: BLUE far→audience", tip.describe())
    }

    @Test
    fun unknownStartIsSetWithoutCountingATip() {
        val hive = HiveStateEstimator(Alliance.RED)
        assertNull(hive.state)
        assertFalse(hive.assumed)
        hive.observe(HiveState.FAR_RAISED, 0, 2)
        val first = hive.observe(HiveState.FAR_RAISED, 10 * ms, 2)

        assertNotNull(first)
        assertFalse(first!!.isTip)
        assertEquals(HiveState.FAR_RAISED, hive.state)
        assertEquals(0, hive.tipCount)
        assertEquals("HIVE STATE: RED far CELL raised (first confirmed sighting)", first.describe())
    }

    @Test
    fun tipAgainstAnUnseenPriorAndAfterALongGapSaysSo() {
        val hive = HiveStateEstimator(Alliance.RED, HiveState.AUDIENCE_RAISED)
        val tip = hive.observe(HiveState.FAR_RAISED, 0, 1)!!
        assertTrue(tip.fromAssumedPrior)
        assertTrue(tip.describe().contains("assumed from match setup"))

        hive.observe(HiveState.FAR_RAISED, 100 * ms, 1)
        val later = hive.observe(HiveState.AUDIENCE_RAISED, 2_100 * ms, 1)!!
        assertEquals(2_000.0, later.unobservedMs!!, 1e-9)
        assertEquals("HIVE TIP: RED far→audience (last seen in the old state 2000 ms earlier)", later.describe())
        assertEquals(2, hive.tipCount)
    }
}
