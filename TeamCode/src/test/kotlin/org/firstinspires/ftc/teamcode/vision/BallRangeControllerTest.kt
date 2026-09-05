package org.firstinspires.ftc.teamcode.vision

import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue
import org.junit.After
import org.junit.Before
import org.junit.Test

class BallRangeControllerTest {

    private val controller = BallRangeController()
    private lateinit var saved: List<Double>

    @Before
    fun captureConfig() {
        saved = listOf(
            BallApproachConfig.kP,
            BallApproachConfig.kD,
            BallApproachConfig.targetTyDegrees,
            BallApproachConfig.maxForwardPower,
            BallApproachConfig.minForwardPower,
            BallApproachConfig.deadbandDegrees,
            BallApproachConfig.alignGateDegrees,
        )
        BallApproachConfig.kP = 0.05
        BallApproachConfig.kD = 0.0
        BallApproachConfig.targetTyDegrees = -10.0
        BallApproachConfig.maxForwardPower = 0.30
        BallApproachConfig.minForwardPower = 0.08
        BallApproachConfig.deadbandDegrees = 1.0
        BallApproachConfig.alignGateDegrees = 25.0
    }

    @After
    fun restoreConfig() {
        BallApproachConfig.kP = saved[0]
        BallApproachConfig.kD = saved[1]
        BallApproachConfig.targetTyDegrees = saved[2]
        BallApproachConfig.maxForwardPower = saved[3]
        BallApproachConfig.minForwardPower = saved[4]
        BallApproachConfig.deadbandDegrees = saved[5]
        BallApproachConfig.alignGateDegrees = saved[6]
    }

    @Test
    fun aBallFartherThanTheStandoffDrivesForward() {
        // Farther away means higher in frame, so ty sits above the setpoint.
        val output = controller.update(DT, tyDegrees = -4.0, txDegrees = 0.0)

        assertTrue(output > 0.0, "expected forward power, got $output")
        assertFalse(controller.atTarget)
    }

    @Test
    fun aBallCloserThanTheStandoffBacksOut() {
        val output = controller.update(DT, tyDegrees = -16.0, txDegrees = 0.0)

        assertTrue(output < 0.0, "expected reverse power, got $output")
    }

    @Test
    fun insideTheDeadbandItStopsAndReportsAtTarget() {
        val output = controller.update(DT, tyDegrees = -10.5, txDegrees = 0.0)

        assertEquals(0.0, output)
        assertTrue(controller.atTarget)
    }

    @Test
    fun aLostTargetStopsDeadAndClearsState() {
        controller.update(DT, tyDegrees = -4.0, txDegrees = 0.0)

        val output = controller.update(DT, tyDegrees = null, txDegrees = null)

        assertEquals(0.0, output)
        assertFalse(controller.atTarget)
        assertTrue(controller.lastTyDegrees.isNaN())
    }

    @Test
    fun aNonFiniteTyIsTreatedAsNoTarget() {
        val output = controller.update(DT, tyDegrees = Double.NaN, txDegrees = 0.0)

        assertEquals(0.0, output)
        assertTrue(controller.lastTyDegrees.isNaN())
    }

    @Test
    fun outputIsClampedToMaxForwardPower() {
        val output = controller.update(DT, tyDegrees = 60.0, txDegrees = 0.0)

        assertEquals(BallApproachConfig.maxForwardPower, output)
    }

    @Test
    fun theFrictionFloorLiftsATinyOutputJustOutsideTheDeadband() {
        BallApproachConfig.kP = 0.001

        val output = controller.update(DT, tyDegrees = -8.5, txDegrees = 0.0)

        assertEquals(BallApproachConfig.minForwardPower, output)
    }

    @Test
    fun theShippedZeroGainDoesNotCreep() {
        // The default kP is 0.0 precisely so the binding is inert until tuned;
        // the friction floor must not resurrect an output the PID never asked for.
        BallApproachConfig.kP = 0.0
        BallApproachConfig.kD = 0.0

        val output = controller.update(DT, tyDegrees = 30.0, txDegrees = 0.0)

        assertEquals(0.0, output)
    }

    @Test
    fun aBallWellOffTheNoseDoesNotDriveForwardYet() {
        val output = controller.update(DT, tyDegrees = -4.0, txDegrees = 40.0)

        assertEquals(0.0, output)
    }

    @Test
    fun theGateOpensOnceTheAimAssistHasBroughtTheBallRound() {
        val gated = controller.update(DT, tyDegrees = -4.0, txDegrees = 40.0)
        val open = controller.update(DT, tyDegrees = -4.0, txDegrees = 5.0)

        assertEquals(0.0, gated)
        assertTrue(open > 0.0, "expected forward power once aligned, got $open")
    }

    @Test
    fun aMissingTxGatesForwardMotion() {
        val output = controller.update(DT, tyDegrees = -4.0, txDegrees = null)

        assertEquals(0.0, output)
    }

    @Test
    fun nonFiniteGainsFallBackToAFinitePower() {
        BallApproachConfig.kP = Double.NaN

        val output = controller.update(DT, tyDegrees = -4.0, txDegrees = 0.0)

        assertTrue(output.isFinite(), "expected a finite power, got $output")
        assertEquals(0.0, output)
    }

    @Test
    fun outputMagnitudeGrowsWithRangeError() {
        val near = abs(controller.update(DT, tyDegrees = -8.0, txDegrees = 0.0))
        controller.reset()
        val far = abs(controller.update(DT, tyDegrees = -4.0, txDegrees = 0.0))

        assertTrue(far > near, "expected $far > $near")
    }

    private companion object {
        const val DT = 0.02
    }
}
