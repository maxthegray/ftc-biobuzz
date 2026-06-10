package org.firstinspires.ftc.teamcode.core.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PIDFControllerTest {

    @Test
    fun proportionalTermTracksError() {
        val controller = PIDFController(PIDFGains(kP = 2.0))
        assertEquals(20.0, controller.calculate(0.02, measurement = 0.0, targetPosition = 10.0), 1e-9)
        assertEquals(-4.0, controller.calculate(0.02, measurement = 12.0, targetPosition = 10.0), 1e-9)
    }

    @Test
    fun feedforwardFollowsSetpointVelocity() {
        val controller = PIDFController(PIDFGains(kS = 0.1, kV = 2.0, kG = 0.05))
        // No error: output is pure feedforward.
        assertEquals(
            0.1 + 2.0 * 1.5 + 0.05,
            controller.calculate(0.02, 0.0, 0.0, targetVelocity = 1.5),
            1e-9,
        )
        // kS flips sign with the commanded direction; kG does not.
        assertEquals(
            -0.1 - 2.0 * 1.5 + 0.05,
            controller.calculate(0.02, 0.0, 0.0, targetVelocity = -1.5),
            1e-9,
        )
    }

    @Test
    fun integralAccumulatesAndIsClampedByIMax() {
        val controller = PIDFController(PIDFGains(kI = 1.0, iMax = 0.5))
        var output = 0.0
        repeat(1000) {
            output = controller.calculate(0.02, measurement = 0.0, targetPosition = 100.0)
        }
        // kP = 0, so output is the integral term alone — capped at iMax.
        assertEquals(0.5, output, 1e-9)
    }

    @Test
    fun derivativeOpposesGrowingError() {
        val controller = PIDFController(PIDFGains(kD = 1.0))
        controller.calculate(0.02, measurement = 0.0, targetPosition = 0.0)
        // Error went 0 → 1 in 0.02 s: derivative = 50.
        assertEquals(50.0, controller.calculate(0.02, measurement = 0.0, targetPosition = 1.0), 1e-9)
    }

    @Test
    fun resetClearsIntegralAndDerivativeState() {
        val controller = PIDFController(PIDFGains(kI = 1.0, kD = 1.0))
        repeat(10) { controller.calculate(0.02, 0.0, 5.0) }
        controller.reset()
        // First call after reset: no derivative kick, integral starts over.
        val output = controller.calculate(0.02, 0.0, 5.0)
        assertEquals(5.0 * 0.02, output, 1e-9)
    }

    @Test
    fun liveGainChangesTakeEffect() {
        val gains = PIDFGains(kP = 1.0)
        val controller = PIDFController(gains)
        assertEquals(10.0, controller.calculate(0.02, 0.0, 10.0), 1e-9)
        gains.kP = 3.0
        assertTrue(controller.calculate(0.02, 0.0, 10.0) > 29.0)
    }
}
