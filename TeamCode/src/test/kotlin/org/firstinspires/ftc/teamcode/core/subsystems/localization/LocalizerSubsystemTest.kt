package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.subsystems.drive.fakeFollower
import org.firstinspires.ftc.teamcode.core.estimation.CorrectionResult
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class LocalizerSubsystemTest {

    private val clock = FakeClock(start = 0L)
    private val follower = fakeFollower()
    private val events = mutableListOf<String>()
    private val localizer = LocalizerSubsystem(follower, clock, onEvent = events::add)

    @After
    fun restoreConfig() {
        LocalizerConfig.watchdogEnabled = true
        LocalizerConfig.frozenPoseTicks = 25
    }

    @Test
    fun correctionPreservesStraightLineMotionSinceMeasurement() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(200_000_000L, Pose2d(5.0, 0.0, 0.0))

        val result = applyUngated(Pose2d(100.0, 10.0, 0.0), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose2d(Pose2d(105.0, 10.0, 0.0), follower.pose)
    }

    @Test
    fun correctionPreservesPureRotationSinceMeasurement() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose2d(0.0, 0.0, PI / 2.0))

        val result = applyUngated(Pose2d(20.0, 30.0, Math.toRadians(10.0)), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose2d(Pose2d(20.0, 30.0, Math.toRadians(100.0)), follower.pose)
    }

    @Test
    fun correctionComposesTranslationInMeasuredFrame() {
        sample(0L, Pose2d(10.0, 0.0, PI / 2.0))
        sample(100_000_000L, Pose2d(10.0, 5.0, PI))

        val result = applyUngated(Pose2d(100.0, 100.0, 0.0), 0L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose2d(Pose2d(105.0, 100.0, PI / 2.0), follower.pose)
    }

    @Test
    fun correctionRejectsStaleTimestamp() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(1_000_000_000L, Pose2d(10.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose2d(100.0, 0.0, 0.0),
            timestampNanos = 0L,
            maxAgeNanos = 500_000_000L,
        )

        assertEquals(CorrectionResult.STALE, result)
        assertPose2d(Pose2d(10.0, 0.0, 0.0), follower.pose)
        assertTrue(events.any { "stale" in it })
    }

    @Test
    fun correctionReportsMissingHistory() {
        // No samples recorded at all.
        val result = localizer.applyCorrection(Pose2d(1.0, 1.0, 0.0), 0L)

        assertEquals(CorrectionResult.NO_HISTORY, result)
    }

    @Test
    fun correctionUsesInterpolatedHistoricalPose() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose2d(10.0, 0.0, 0.0))
        sample(200_000_000L, Pose2d(20.0, 0.0, 0.0))

        val result = applyUngated(Pose2d(100.0, 0.0, 0.0), 50_000_000L)

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose2d(Pose2d(115.0, 0.0, 0.0), follower.pose)
    }

    @Test
    fun correctionRejectsJumpBeyondPositionGate() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose2d(5.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose2d(100.0, 0.0, 0.0),
            timestampNanos = 0L,
            blend = 1.0,
            maxJumpInches = 12.0,
            maxJumpRadians = Double.MAX_VALUE,
        )

        assertEquals(CorrectionResult.REJECTED_JUMP, result)
        assertPose2d(Pose2d(5.0, 0.0, 0.0), follower.pose)
        assertTrue(events.any { "jump" in it })
    }

    @Test
    fun correctionRejectsJumpBeyondHeadingGate() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose2d(0.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose2d(0.0, 0.0, Math.toRadians(90.0)),
            timestampNanos = 0L,
            blend = 1.0,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Math.toRadians(30.0),
        )

        assertEquals(CorrectionResult.REJECTED_JUMP, result)
        assertPose2d(Pose2d(0.0, 0.0, 0.0), follower.pose)
    }

    @Test
    fun blendAppliesFractionOfCorrection() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose2d(0.0, 0.0, 0.0))

        val result = localizer.applyCorrection(
            measured = Pose2d(10.0, 4.0, Math.toRadians(20.0)),
            timestampNanos = 0L,
            blend = 0.5,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Double.MAX_VALUE,
        )

        assertEquals(CorrectionResult.APPLIED, result)
        assertPose2d(Pose2d(5.0, 2.0, Math.toRadians(10.0)), follower.pose)
    }

    @Test
    fun repeatedBlendedCorrectionsConvergeOnMeasurement() {
        sample(0L, Pose2d(0.0, 0.0, 0.0))
        repeat(8) { i ->
            val t = (i + 1) * 100_000_000L
            val p = follower.pose
            sample(t, Pose2d(p.x, p.y, p.heading))
            localizer.applyCorrection(
                measured = Pose2d(10.0, 0.0, 0.0),
                timestampNanos = t,
                blend = 0.5,
                maxJumpInches = Double.MAX_VALUE,
                maxJumpRadians = Double.MAX_VALUE,
            )
        }
        assertTrue("expected convergence, got ${follower.pose.x}", follower.pose.x > 9.5)
    }

    // ---------------------------------------------------------------- watchdog

    private class WatchdogHarness(
        following: Boolean = true,
    ) {
        val follower = fakeFollower()
        var faults = 0
        val events = mutableListOf<String>()
        val localizer = LocalizerSubsystem(
            follower,
            onEvent = events::add,
            isFollowing = { following },
            onFault = { faults++ },
        )

        fun setPose(x: Double, y: Double, heading: Double) {
            follower.setPose(Pose(x, y, heading))
        }
    }

    @Test
    fun watchdogTripsOnNonFinitePose() {
        val h = WatchdogHarness()
        h.setPose(Double.NaN, 0.0, 0.0)

        h.localizer.periodic()

        assertEquals(1, h.faults)
        assertTrue(h.localizer.fault!!.contains("non-finite"))
        assertTrue(h.localizer.health().startsWith("FAULT"))
        assertTrue(h.events.any { "LOCALIZER FAULT" in it })

        // Latched: further ticks don't refire the policy callback.
        h.localizer.periodic()
        assertEquals(1, h.faults)
    }

    @Test
    fun watchdogTripsOnPoseFrozenWhileFollowing() {
        LocalizerConfig.frozenPoseTicks = 5
        val h = WatchdogHarness(following = true)
        h.setPose(10.0, 20.0, 1.0)

        h.localizer.periodic() // primes the last-pose comparison
        repeat(4) { h.localizer.periodic() }
        assertNull(h.localizer.fault)

        h.localizer.periodic()
        assertEquals(1, h.faults)
        assertTrue(h.localizer.fault!!.contains("frozen"))
    }

    @Test
    fun watchdogIgnoresFrozenPoseWhenNotFollowing() {
        LocalizerConfig.frozenPoseTicks = 5
        val h = WatchdogHarness(following = false)
        h.setPose(10.0, 20.0, 1.0)

        repeat(50) { h.localizer.periodic() }

        assertNull(h.localizer.fault)
        assertEquals(0, h.faults)
    }

    @Test
    fun watchdogDoesNotTripWhilePoseIsMoving() {
        LocalizerConfig.frozenPoseTicks = 5
        val h = WatchdogHarness(following = true)

        repeat(50) { i ->
            h.setPose(i * 0.01, 20.0, 1.0)
            h.localizer.periodic()
        }

        assertNull(h.localizer.fault)
        assertEquals("ok", h.localizer.health())
    }

    @Test
    fun watchdogCanBeDisabled() {
        LocalizerConfig.watchdogEnabled = false
        val h = WatchdogHarness()
        h.setPose(Double.NaN, 0.0, 0.0)

        h.localizer.periodic()

        assertNull(h.localizer.fault)
        assertEquals(0, h.faults)
    }

    @Test
    fun watchdogSurvivesThrowingFaultPolicy() {
        val h = WatchdogHarness()
        val localizer = LocalizerSubsystem(
            h.follower,
            onEvent = h.events::add,
            isFollowing = { true },
            onFault = { error("policy blew up") },
        )
        h.setPose(Double.NaN, 0.0, 0.0)

        localizer.periodic() // must not throw

        assertTrue(localizer.fault!!.contains("non-finite"))
    }

    private fun applyUngated(measured: Pose2d, timestampNanos: Long): CorrectionResult =
        localizer.applyCorrection(
            measured = measured,
            timestampNanos = timestampNanos,
            blend = 1.0,
            maxJumpInches = Double.MAX_VALUE,
            maxJumpRadians = Double.MAX_VALUE,
        )

    /** Pose history is sampled in writeHardware, right after Follower.update(). */
    private fun sample(timestampNanos: Long, pose: Pose2d) {
        clock.now = timestampNanos
        follower.setPose(Pose(pose.x, pose.y, pose.heading))
        localizer.writeHardware()
    }

    private fun assertPose2d(expected: Pose2d, actual: Pose) {
        assertEquals(expected.x, actual.x, EPS)
        assertEquals(expected.y, actual.y, EPS)
        assertEquals(expected.heading, actual.heading, EPS)
    }

    private companion object {
        const val EPS = 1e-6
    }
}
