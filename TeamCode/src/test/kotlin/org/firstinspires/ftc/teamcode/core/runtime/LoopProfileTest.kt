package org.firstinspires.ftc.teamcode.core.runtime

import org.junit.Assert.assertEquals
import org.junit.Test

class LoopProfileTest {

    @Test
    fun maximaTrackPeaks() {
        val p = LoopProfile()
        p.periodicNanos = 5_000
        p.periodicNanos = 3_000
        p.recordNanos = 7_000
        p.recordNanos = 4_000
        assertEquals(3_000, p.periodicNanos)
        assertEquals(5_000, p.maxPeriodicNanos)
        assertEquals(4_000, p.recordNanos)
        assertEquals(7_000, p.maxRecordNanos)
    }

    @Test
    fun resetMaximaClearsPeaksButNotLatest() {
        val p = LoopProfile()
        p.schedulerNanos = 9_000
        p.resetMaxima()
        assertEquals(9_000, p.schedulerNanos)
        assertEquals(0, p.maxSchedulerNanos)
        // Next assignment re-seeds the max.
        p.schedulerNanos = 4_000
        assertEquals(4_000, p.maxSchedulerNanos)
    }

    @Test
    fun overheadIsResidueOfTotal() {
        val p = LoopProfile()
        p.clearCachesNanos = 1_000
        p.periodicNanos = 2_000
        p.controlNanos = 3_000
        p.schedulerNanos = 4_000
        p.writeHardwareNanos = 5_000
        p.recordNanos = 1_000
        p.totalNanos = 20_000
        assertEquals(4_000, p.overheadNanos)
    }

    @Test
    fun overheadNeverGoesNegative() {
        val p = LoopProfile()
        p.periodicNanos = 10_000
        p.totalNanos = 5_000
        assertEquals(0, p.overheadNanos)
    }

    @Test
    fun maxOverheadCapturedAtTotalAssignment() {
        val p = LoopProfile()
        p.periodicNanos = 1_000
        p.totalNanos = 11_000
        assertEquals(10_000, p.maxOverheadNanos)
        // Smaller overhead later does not lower the max.
        p.periodicNanos = 1_000
        p.totalNanos = 2_000
        assertEquals(10_000, p.maxOverheadNanos)
    }
}
