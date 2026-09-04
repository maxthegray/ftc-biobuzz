package org.firstinspires.ftc.teamcode.vision

import kotlin.math.abs
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightColorTarget
import org.junit.After
import org.junit.Before
import org.junit.Test

class BallAimControllerTest {

    private val controller = BallAimController()
    private lateinit var saved: List<Double>

    @Before
    fun captureConfig() {
        saved = listOf(
            BallAimConfig.kP,
            BallAimConfig.kD,
            BallAimConfig.maxTurnPower,
            BallAimConfig.minTurnPower,
            BallAimConfig.deadbandDegrees,
            BallAimConfig.minAreaPercent,
        )
        BallAimConfig.kP = 0.02
        BallAimConfig.kD = 0.0
        BallAimConfig.maxTurnPower = 0.45
        BallAimConfig.minTurnPower = 0.06
        BallAimConfig.deadbandDegrees = 1.5
        BallAimConfig.minAreaPercent = 0.15
    }

    @After
    fun restoreConfig() {
        BallAimConfig.kP = saved[0]
        BallAimConfig.kD = saved[1]
        BallAimConfig.maxTurnPower = saved[2]
        BallAimConfig.minTurnPower = saved[3]
        BallAimConfig.deadbandDegrees = saved[4]
        BallAimConfig.minAreaPercent = saved[5]
    }

    @Test
    fun targetRightOfCentreTurnsClockwise() {
        // Limelight tx is positive-right; Pedro turn is CCW-positive, so facing
        // a target on the right means a negative output.
        val output = controller.update(DT, txDegrees = 10.0)

        assertTrue(output < 0.0, "expected CW (negative) turn, got $output")
        assertTrue(controller.hasTarget)
        assertFalse(controller.onTarget)
    }

    @Test
    fun targetLeftOfCentreTurnsCounterClockwise() {
        assertTrue(controller.update(DT, txDegrees = -10.0) > 0.0)
    }

    @Test
    fun noTargetCommandsZeroAndClearsState() {
        controller.update(DT, txDegrees = 20.0)

        assertEquals(0.0, controller.update(DT, txDegrees = null))
        assertFalse(controller.hasTarget)
        assertFalse(controller.onTarget)
        assertEquals(0.0, controller.lastOutput)
    }

    @Test
    fun losingTheTargetDoesNotCarryDerivativeIntoReacquisition() {
        BallAimConfig.kD = 0.5
        controller.update(DT, txDegrees = 20.0)
        controller.update(DT, txDegrees = null)

        // Re-acquiring at the same error must give the pure proportional term:
        // a retained lastError of 20 would add a large derivative kick.
        val output = controller.update(DT, txDegrees = 20.0)

        assertEquals(-0.02 * 20.0, output, TOLERANCE)
    }

    @Test
    fun insideDeadbandIsOnTargetAndStill() {
        val output = controller.update(DT, txDegrees = 1.0)

        assertEquals(0.0, output)
        assertTrue(controller.hasTarget)
        assertTrue(controller.onTarget)
    }

    @Test
    fun outputIsClampedToMaxTurnPower() {
        // 60 deg * 0.02 = 1.2, well past the 0.45 cap.
        assertEquals(-0.45, controller.update(DT, txDegrees = 60.0), TOLERANCE)
    }

    @Test
    fun frictionFloorAppliesJustOutsideTheDeadband() {
        // 2 deg * 0.02 = 0.04, below the 0.06 floor.
        val output = controller.update(DT, txDegrees = 2.0)

        assertEquals(-0.06, output, TOLERANCE)
    }

    @Test
    fun zeroGainDoesNotCreepOnTheFrictionFloor() {
        BallAimConfig.kP = 0.0

        assertEquals(0.0, controller.update(DT, txDegrees = 30.0))
    }

    @Test
    fun nonFiniteGainsFallBackInsteadOfNaNingTheMotors() {
        BallAimConfig.kP = Double.NaN

        val output = controller.update(DT, txDegrees = 30.0)

        assertTrue(output.isFinite(), "expected a finite power, got $output")
        assertEquals(0.0, output)
    }

    @Test
    fun selectsTheLargestTargetSoASecondBallDoesNotDragTheAim() {
        val tx = controller.selectTx(
            targets = listOf(
                LimelightColorTarget(txDegrees = -12.0, tyDegrees = 0.0, areaPercent = 0.4),
                LimelightColorTarget(txDegrees = 5.0, tyDegrees = 0.0, areaPercent = 2.5),
            ),
            primary = null,
        )

        assertEquals(5.0, tx)
    }

    @Test
    fun ignoresBlobsBelowTheMinimumArea() {
        val tx = controller.selectTx(
            targets = listOf(
                LimelightColorTarget(txDegrees = 8.0, tyDegrees = 0.0, areaPercent = 0.01),
            ),
            primary = null,
        )

        assertNull(tx)
    }

    @Test
    fun fallsBackToThePrimaryTargetWhenTheColorListIsEmpty() {
        val tx = controller.selectTx(
            targets = emptyList(),
            primary = LimelightColorTarget(txDegrees = -3.0, tyDegrees = 0.0, areaPercent = 1.0),
        )

        assertEquals(-3.0, tx)
    }

    @Test
    fun noVisibleTargetSelectsNothing() {
        assertNull(controller.selectTx(targets = emptyList(), primary = null))
    }

    @Test
    fun outputMagnitudeGrowsWithError() {
        val near = abs(controller.update(DT, txDegrees = 5.0))
        controller.reset()
        val far = abs(controller.update(DT, txDegrees = 15.0))

        assertTrue(far > near, "expected $far > $near")
    }

    private companion object {
        const val DT = 0.02
        const val TOLERANCE = 1e-9
    }
}
