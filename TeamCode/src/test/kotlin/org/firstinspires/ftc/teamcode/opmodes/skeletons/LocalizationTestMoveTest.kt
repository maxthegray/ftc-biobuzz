package org.firstinspires.ftc.teamcode.opmodes.skeletons

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.firstinspires.ftc.teamcode.opmodes.skeletons.LocalizationTestTeleOp.Companion.COINCIDENT_WAYPOINT_TOLERANCE_INCHES
import org.firstinspires.ftc.teamcode.opmodes.skeletons.LocalizationTestTeleOp.Companion.moveCommand
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/** Localization Test's button moves against the real Pedro drive, including targets it is already at. */
class LocalizationTestMoveTest {

    private val hardware = PedroDriveFixture()
    private val drive = hardware.drive
    private val robot = Robot(hardware.hardwareMap, hardware.clock).also {
        it.register(drive)
        it.start()
    }

    private fun tick() {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    private fun plan(start: Pose, target: Pose, heading: Double? = null) =
        moveCommand(drive, start, target, heading, pathSpeedFraction = 0.3)

    @Test
    fun pedroRejectsOnlyExactlyZeroLengthLines() {
        // Why coincident targets never become paths: an identical target
        // throws (a COMMAND FAULT from a button press), a nearly identical one
        // builds a line with no meaningful direction.
        val p = Pose(10.0, 20.0, 0.0)
        assertThrows(IllegalArgumentException::class.java) { Paths.line(p, p) }
        assertEquals(1e-9, Paths.line(p, Pose(10.0 + 1e-9, 20.0)).curve.length(), 1e-12)
    }

    @Test
    fun identicalPosesDoNothingAndLeaveRunningCommandsAlone() {
        val unrelated = infinite {}.requiring(Any())
        Scheduler.schedule(unrelated)
        val here = Pose(10.0, 20.0, 1.0)
        hardware.localizer.measuredPose = here

        assertNull(plan(here, here))
        assertNull(plan(here, here, heading = here.heading()))
        tick()

        assertTrue(Scheduler.isScheduled(unrelated))
        assertEquals("IDLE", drive.driveModeName)
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun positionsWithinTheToleranceAreTheSameWaypoint() {
        val here = Pose(10.0, 20.0, 0.0)
        val jitter = COINCIDENT_WAYPOINT_TOLERANCE_INCHES * 0.8
        assertNull(plan(here, Pose(10.0 + jitter, 20.0, 0.0)))
        assertNull(plan(here, Pose(10.0 + 1e-9, 20.0 - 1e-9, 0.0)))

        val justOutside = Pose(10.0 + COINCIDENT_WAYPOINT_TOLERANCE_INCHES * 1.2, 20.0, 0.0)
        hardware.localizer.measuredPose = here
        val move = plan(here, justOutside)
        assertNotNull(move)
        Scheduler.schedule(move)
        tick()
        assertEquals("FOLLOWING", drive.driveModeName)
        assertTrue(hardware.powers().all { it.isFinite() })
    }

    @Test
    fun samePositionWithADifferentHeadingOnlyTurns() {
        val here = Pose(10.0, 20.0, 0.0)
        hardware.localizer.measuredPose = here
        val move = plan(here, Pose(10.0, 20.0, 0.0), heading = Math.PI / 2)!!
        Scheduler.schedule(move)
        tick()
        assertEquals("HOLDING", drive.driveModeName)

        hardware.localizer.measuredPose = Pose(10.0, 20.0, Math.PI / 2)
        tick()
        tick()
        assertFalse(Scheduler.isScheduled(move))
        assertEquals(0, robot.commandFaultCount)
        // A heading within the hold tolerance is not a turn.
        assertNull(plan(here, here, heading = 1e-4))
    }

    @Test
    fun aRealMoveWithAHeadingFollowsThenTurns() {
        val here = Pose(0.0, 0.0, 0.0)
        hardware.localizer.measuredPose = here
        val move = plan(here, Pose(24.0, 0.0), heading = Math.PI / 2)!!
        Scheduler.schedule(move)
        tick()
        assertEquals("FOLLOWING", drive.driveModeName)

        hardware.localizer.measuredPose = Pose(24.0, 0.0, 0.0)
        repeat(3) { tick() }
        assertTrue(Scheduler.isScheduled(move))
        assertEquals("HOLDING", drive.driveModeName)
        hardware.localizer.measuredPose = Pose(24.0, 0.0, Math.PI / 2)
        repeat(2) { tick() }
        assertFalse(Scheduler.isScheduled(move))
        assertEquals(0, robot.commandFaultCount)
    }
}
