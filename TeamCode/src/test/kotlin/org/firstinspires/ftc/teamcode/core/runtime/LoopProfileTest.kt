package org.firstinspires.ftc.teamcode.core.runtime

import org.junit.Assert.assertEquals
import org.junit.Test

class LoopProfileTest {

    @Test
    fun maximaTrackPeaks() {
        val p = LoopProfile()
        p[LoopPhase.PERIODIC] = 5_000
        p[LoopPhase.PERIODIC] = 3_000
        p[LoopPhase.RECORD] = 7_000
        p[LoopPhase.RECORD] = 4_000
        assertEquals(3_000, p[LoopPhase.PERIODIC])
        assertEquals(5_000, p.max(LoopPhase.PERIODIC))
        assertEquals(4_000, p[LoopPhase.RECORD])
        assertEquals(7_000, p.max(LoopPhase.RECORD))
    }

    @Test
    fun resetMaximaClearsPeaksButNotLatest() {
        val p = LoopProfile()
        p[LoopPhase.SCHEDULER] = 9_000
        p.resetMaxima()
        assertEquals(9_000, p[LoopPhase.SCHEDULER])
        assertEquals(0, p.max(LoopPhase.SCHEDULER))
        // Next assignment re-seeds the max.
        p[LoopPhase.SCHEDULER] = 4_000
        assertEquals(4_000, p.max(LoopPhase.SCHEDULER))
    }

    @Test
    fun overheadIsResidueOfTotal() {
        val p = LoopProfile()
        p[LoopPhase.CLEAR_CACHES] = 1_000
        p[LoopPhase.PERIODIC] = 2_000
        p[LoopPhase.CONTROL] = 3_000
        p[LoopPhase.SCHEDULER] = 4_000
        p[LoopPhase.WRITE_HARDWARE] = 5_000
        p[LoopPhase.RECORD] = 1_000
        p.totalNanos = 20_000
        assertEquals(4_000, p.overheadNanos)
    }

    @Test
    fun overheadNeverGoesNegative() {
        val p = LoopProfile()
        p[LoopPhase.PERIODIC] = 10_000
        p.totalNanos = 5_000
        assertEquals(0, p.overheadNanos)
    }

    @Test
    fun maxOverheadCapturedAtTotalAssignment() {
        val p = LoopProfile()
        p[LoopPhase.PERIODIC] = 1_000
        p.totalNanos = 11_000
        assertEquals(10_000, p.maxOverheadNanos)
        // Smaller overhead later does not lower the max.
        p[LoopPhase.PERIODIC] = 1_000
        p.totalNanos = 2_000
        assertEquals(10_000, p.maxOverheadNanos)
    }

    @Test
    fun everyPhaseHasAStableLabel() {
        // WPILOG channel names (loop/<label>Nanos) and the analyzer depend on
        // these exact stems — renaming one breaks post-match tooling.
        assertEquals(
            listOf(
                "clearCaches", "periodic", "input", "control",
                "scheduler", "writeHardware", "telemetry", "record",
            ),
            LoopPhase.entries.map { it.label },
        )
    }
}
