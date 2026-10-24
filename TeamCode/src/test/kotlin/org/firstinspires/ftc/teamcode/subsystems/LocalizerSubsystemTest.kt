package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.logging.Events
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class LocalizerSubsystemTest {

    private val events = mutableListOf<String>()

    @Before
    fun captureEvents() {
        Events.sink = events::add
    }

    @After
    fun restoreConfig() {
        Events.reset()
        LocalizerConfig.frozenPoseTicks = 25
    }

    // ---------------------------------------------------------------- watchdog

    private class WatchdogHarness(
        following: Boolean = true,
    ) {
        val follower = PedroDriveFixture(tuned = false).follower
        var faults = 0
        val events = mutableListOf<String>()
        init { Events.sink = events::add }
        val localizer = LocalizerSubsystem(
            follower,
            isFollowing = { following },
            onFault = { faults++ },
            onStatusFault = { faults++ },
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
    fun watchdogSurvivesThrowingFaultPolicy() {
        val h = WatchdogHarness()
        val localizer = LocalizerSubsystem(
            h.follower,
            isFollowing = { true },
            onFault = { error("policy blew up") },
            onStatusFault = { error("policy blew up") },
        ).also(::initialized)
        h.setPose(Double.NaN, 0.0, 0.0)

        localizer.periodic() // must not throw

        assertTrue(localizer.fault!!.contains("non-finite"))
    }
}

/** Init the localizer on an empty hardware map and confirm its start pose, as the init loop does. */
private fun initialized(localizer: LocalizerSubsystem) {
    localizer.init(com.qualcomm.robotcore.hardware.HardwareMap(null, null))
    localizer.initPeriodic()
    assertTrue(localizer.ready)
}
