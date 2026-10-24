package org.firstinspires.ftc.teamcode.subsystems

import kotlin.math.PI
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.Companion.wrapToTravel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class TurretWrapTest {

    private val eps = 1e-9

    @Test
    fun targetInsideTravelIsUnchanged() {
        assertEquals(0.5, wrapToTravel(0.5, 0.0, -PI, PI)!!, eps)
    }

    @Test
    fun wrapsToTheEquivalentNearestTheCurrentAngle() {
        assertEquals(-3.0 + 2 * PI, wrapToTravel(-3.0, 3.0, -1.5 * PI, 1.5 * PI)!!, eps)
        assertEquals(2 * PI, wrapToTravel(0.0, 6.0, -4 * PI, 4 * PI)!!, eps)
    }

    @Test
    fun outsideTravelIsNull() {
        assertNull(wrapToTravel(2.0, 0.0, -1.0, 1.0))
    }
}
