package org.firstinspires.ftc.teamcode.core.util

import org.junit.Assert.assertEquals
import org.junit.Test

class DeadbandTest {

    private val eps = 1e-12

    @Test
    fun zeroStaysZero() {
        assertEquals(0.0, applyDeadband(0.0, 0.05), eps)
    }

    @Test
    fun insideDeadbandReadsZero() {
        assertEquals(0.0, applyDeadband(0.04, 0.05), eps)
        assertEquals(0.0, applyDeadband(-0.04, 0.05), eps)
    }

    @Test
    fun edgeOfDeadbandReadsZero() {
        // No jump at the deadband boundary: output starts from 0 exactly there.
        assertEquals(0.0, applyDeadband(0.05, 0.05), eps)
    }

    @Test
    fun fullDeflectionStaysFull() {
        assertEquals(1.0, applyDeadband(1.0, 0.05), eps)
        assertEquals(-1.0, applyDeadband(-1.0, 0.05), eps)
    }

    @Test
    fun remainingRangeIsRescaled() {
        // Halfway between deadband edge and full deflection -> 0.5.
        assertEquals(0.5, applyDeadband(0.525, 0.05), eps)
        assertEquals(-0.5, applyDeadband(-0.525, 0.05), eps)
    }

    @Test
    fun zeroDeadbandIsIdentity() {
        assertEquals(0.7, applyDeadband(0.7, 0.0), eps)
        assertEquals(-0.7, applyDeadband(-0.7, 0.0), eps)
    }
}
