package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.RumbleAlert
import org.firstinspires.ftc.teamcode.core.RumbleAlerts
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.BallTrackingTestTeleOp
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.HiveTagSurveyTeleOp
import org.firstinspires.ftc.teamcode.opmodes.diagnostics.LimelightAprilTagTestTeleOp
import org.firstinspires.ftc.teamcode.opmodes.skeletons.ExampleAuto
import org.firstinspires.ftc.teamcode.opmodes.skeletons.LocalizationTestTeleOp
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class BiobuzzRumbleTest {
    @Suppress("UNCHECKED_CAST")
    private fun alerts(opMode: OpModeBase): List<RumbleAlert> =
        OpModeBase::class.java.getDeclaredMethod("getRumbleAlerts").apply {
            isAccessible = true
        }.invoke(opMode) as List<RumbleAlert>

    private fun timer() = RumbleAlerts(alerts(DriveOnlyTeleOp()))

    @Test
    fun cuesFireAtOneMinuteAndTwentySecondsLeftOnlyOnce() {
        val timer = timer()
        assertNull(timer.poll(0))
        assertNull(timer.poll(59_999_999_999L))
        assertEquals(2, timer.poll(60_000_000_000L)?.blips)
        assertNull(timer.poll(60_000_000_000L))
        assertNull(timer.poll(90_000_000_000L))
        assertNull(timer.poll(99_999_999_999L))
        assertEquals(3, timer.poll(100_000_000_000L)?.blips)
        assertNull(timer.poll(100_000_000_000L))
        assertNull(timer.poll(119_999_999_999L))
    }

    @Test
    fun aLateLoopStillEmitsTheCrossedCue() {
        val timer = timer()
        assertNull(timer.poll(59_000_000_000L))
        assertEquals(2, timer.poll(60_020_000_000L)?.blips)
        assertNull(timer.poll(61_000_000_000L))
        assertEquals(3, timer.poll(100_020_000_000L)?.blips)
    }

    @Test
    fun skippingBothThresholdsEmitsOnlyTheLatestCue() {
        val timer = timer()
        assertNull(timer.poll(59_000_000_000L))
        assertEquals(3, timer.poll(105_000_000_000L)?.blips)
        assertNull(timer.poll(105_020_000_000L))
    }

    @Test
    fun overdueCuesAreSuppressedAtAndAfterTheMatchEnd() {
        assertEquals(3, timer().poll(119_999_999_999L)?.blips)
        for (elapsed in listOf(120_000_000_000L, 125_000_000_000L)) {
            val timer = timer()
            assertNull(timer.poll(elapsed))
            assertNull(timer.poll(elapsed + 1))
        }
    }

    @Test
    fun eachRunStartsWithFreshAlertProgress() {
        val opMode = DriveOnlyTeleOp()
        repeat(2) {
            val timer = RumbleAlerts(alerts(opMode))
            assertNull(timer.poll(0))
            assertEquals(2, timer.poll(60_000_000_000L)?.blips)
            assertEquals(3, timer.poll(100_000_000_000L)?.blips)
        }
    }

    @Test
    fun autonomousAndDiagnosticsHaveNoMatchAlerts() {
        for (opMode in listOf(
            TwoTipAutoRed(), ExampleAuto(), BallTrackingTestTeleOp(),
            LimelightAprilTagTestTeleOp(), HiveTagSurveyTeleOp(), LocalizationTestTeleOp(),
        )) {
            assertTrue(opMode.javaClass.simpleName, alerts(opMode).isEmpty())
        }
        val timer = RumbleAlerts(emptyList())
        for (elapsed in listOf(0L, 60_000_000_000L, 100_000_000_000L, 125_000_000_000L)) {
            assertNull(timer.poll(elapsed))
        }
    }
}
