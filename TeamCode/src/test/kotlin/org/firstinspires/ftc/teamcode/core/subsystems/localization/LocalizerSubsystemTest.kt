package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.math.Pose
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class LocalizerSubsystemTest {

    private val clock = FakeClock(start = 0L)
    private val follower = PedroDriveFixture(tuned = false).follower
    private val events = mutableListOf<String>()
    private val localizer = LocalizerSubsystem(follower, clock, onEvent = events::add)

    @After
    fun restoreConfig() {
        LocalizerConfig.watchdogEnabled = true
        LocalizerConfig.frozenPoseTicks = 25
    }

    @Test
    fun poseAtInterpolatesTheSampledHistory() {
        sample(0L, Pose(0.0, 0.0, 0.0))
        sample(100_000_000L, Pose(10.0, 0.0, PI / 2.0))

        assertPose2d(Pose(5.0, 0.0, PI / 4.0), localizer.poseAt(50_000_000L)!!)
        assertNull(localizer.poseAt(200_000_000L))
    }

    // ---------------------------------------------------------------- watchdog

    private class WatchdogHarness(
        following: Boolean = true,
    ) {
        val follower = PedroDriveFixture(tuned = false).follower
        var faults = 0
        val events = mutableListOf<String>()
        val localizer = LocalizerSubsystem(
            follower,
            onEvent = events::add,
            isFollowing = { following },
            onFault = { faults++ },
        ).also(::initialized)

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
        ).also(::initialized)
        h.setPose(Double.NaN, 0.0, 0.0)

        localizer.periodic() // must not throw

        assertTrue(localizer.fault!!.contains("non-finite"))
    }

    /** Pose history is sampled in writeHardware, right after Follower.update(). */
    private fun sample(timestampNanos: Long, pose: Pose) {
        clock.now = timestampNanos
        follower.setPose(pose)
        localizer.writeHardware()
    }

    private fun assertPose2d(expected: Pose, actual: Pose) {
        assertEquals(expected.x(), actual.x(), EPS)
        assertEquals(expected.y(), actual.y(), EPS)
        assertEquals(expected.heading(), actual.heading(), EPS)
    }

    private companion object {
        const val EPS = 1e-6
    }
}

/** Init the localizer on an empty hardware map and confirm its start pose, as the init loop does. */
private fun initialized(localizer: LocalizerSubsystem) {
    localizer.init(com.qualcomm.robotcore.hardware.HardwareMap(null, null))
    localizer.initPeriodic()
    assertTrue(localizer.ready)
}
