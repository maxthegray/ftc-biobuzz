package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

/** Path, hold and turn commands against the real Pedro 3 follower, Foresight and Ivy. */
class DriveCommandTest {

    private class Harness(tuned: Boolean = true) {
        val hardware = PedroDriveFixture(tuned)
        val drive = hardware.drive
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        val start = Pose(0.0, 0.0, 0.0)
        val end = Pose(48.0, 0.0, 0.0)

        init {
            hardware.localizer.measuredPose = start
            robot.register(drive)
            robot.start()
        }

        fun place(pose: Pose) {
            hardware.localizer.measuredPose = pose
        }

        fun tick(ms: Double = 20.0) {
            hardware.clock.advanceMs(ms)
            robot.loop()
        }

        fun line() = Paths.line(start, end).constant(0.0)
    }

    @Test
    fun followEndsAtPedrosParametricEndWhichIsNotArrival() {
        val h = Harness()
        val follow = h.drive.followCommand(h.line())
        Scheduler.schedule(follow)
        h.tick()
        assertTrue(Scheduler.isScheduled(follow))
        assertEquals("FOLLOWING", h.drive.driveModeName)
        assertTrue(h.hardware.powers().any { it != 0.0 })

        // Level with the end of the path but 3 inches off it.
        h.place(Pose(48.0, 3.0, 0.0))
        repeat(3) { h.tick() }

        assertFalse(Scheduler.isScheduled(follow))
        assertEquals("IDLE", h.drive.driveModeName)
        assertFalse(h.drive.atPose(h.end))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun followWithHoldEndKeepsHoldingAfterTheCommandEnds() {
        val h = Harness()
        val follow = h.drive.followCommand(h.line(), holdEnd = true)
        Scheduler.schedule(follow)
        h.tick()
        h.place(h.end)
        repeat(3) { h.tick() }
        assertFalse(Scheduler.isScheduled(follow))
        assertEquals("HOLDING", h.drive.driveModeName)
    }

    @Test
    fun cancellingAFollowStopsTheMotorsImmediately() {
        val h = Harness()
        val follow = h.drive.followCommand(h.line())
        Scheduler.schedule(follow)
        h.tick()
        assertTrue(h.hardware.powers().any { it != 0.0 })

        Scheduler.cancel(follow)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        h.tick()
        assertEquals("IDLE", h.drive.driveModeName)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun cancellingTheEnclosingGroupStopsTheFollow() {
        val h = Harness()
        var nextRan = false
        val routine = sequential(h.drive.followCommand(h.line()), instant { nextRan = true })
        Scheduler.schedule(routine)
        h.tick()
        Scheduler.cancel(routine)
        h.tick()
        assertFalse(nextRan)
        assertEquals("IDLE", h.drive.driveModeName)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun driverTakeoverInterruptsThePathAndDrivesInTheSameTick() {
        val h = Harness()
        val follow = h.drive.followCommand(h.line())
        Scheduler.schedule(follow)
        h.tick()

        val takeover = h.drive.teleopCommand(priority = CommandPriorities.DRIVER_OVERRIDE) {
            TeleopInput(0.0, 0.0, 0.0, turnPower = 0.4)
        }
        Scheduler.schedule(takeover)
        assertFalse(Scheduler.isScheduled(follow))
        h.tick()
        assertEquals("TELEOP", h.drive.driveModeName)
        assertArrayEquals(doubleArrayOf(-0.4, 0.4, -0.4, 0.4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun defaultTeleopCannotPreemptARunningPath() {
        val h = Harness()
        val teleop = h.drive.teleopCommand { TeleopInput(1.0, 0.0, 0.0) }
        h.drive.defaultCommand = teleop
        h.tick()
        val follow = h.drive.followCommand(h.line())
        Scheduler.schedule(follow)
        repeat(3) { h.tick() }
        assertTrue(Scheduler.isScheduled(follow))
        assertFalse(Scheduler.isScheduled(teleop))
        assertEquals("FOLLOWING", h.drive.driveModeName)
    }

    @Test
    fun pathCommandsRefuseToStartWithoutForesightAndTheRobotHaltsWithAReason() {
        val h = Harness(tuned = false)
        assertThrows(IllegalStateException::class.java) { h.drive.followCommand(h.line()).start() }

        // Scheduling from a binding happens in the input phase, inside the robot's fault boundary.
        h.robot.loop(input = { Scheduler.schedule(h.drive.holdCommand(h.end)) })
        assertEquals(1, h.robot.commandFaultCount)
        assertTrue(h.robot.lastCommandFault!!.message!!.contains("Foresight"))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun pathProgressLatchesAcrossSegmentsAndResetsWhenIdle() {
        val h = Harness()
        val mid = Pose(24.0, 0.0, 0.0)
        val path = Paths.path(Paths.line(h.start, mid), Paths.line(mid, h.end)).constant(0.0)
        Scheduler.schedule(h.drive.followCommand(path))
        h.tick()
        assertEquals(0.0, h.drive.pathProgress(), 0.05)

        h.place(Pose(12.0, 0.0, 0.0))
        h.tick()
        assertEquals(0.25, h.drive.pathProgress(), 0.05)

        // Measured progress moving backwards never lowers the latch.
        h.place(Pose(6.0, 0.0, 0.0))
        h.tick()
        assertEquals(0.25, h.drive.pathProgress(), 0.05)

        h.place(h.end)
        repeat(4) { h.tick() }
        assertEquals(0.0, h.drive.pathProgress(), 0.0)
        assertEquals("IDLE", h.drive.driveModeName)
    }

    @Test
    fun markersFireOnceWhenReachedAndAreDroppedWhenThePathIsCancelled() {
        val h = Harness()
        var reached = 0
        var unreached = 0
        val routine = deadline(
            h.drive.followCommand(h.line()),
            sequential(waitUntil { h.drive.pathProgress() >= 0.25 }, instant { reached++ }),
            sequential(waitUntil { h.drive.pathProgress() >= 0.9 }, instant { unreached++ }),
        )
        Scheduler.schedule(routine)
        h.tick()
        h.place(Pose(24.0, 0.0, 0.0))
        repeat(5) { h.tick() }
        assertEquals(1, reached)

        Scheduler.cancel(routine)
        h.place(h.end)
        repeat(5) { h.tick() }
        assertEquals(1, reached)
        assertEquals(0, unreached)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun aRaceTimeoutInterruptsTheStepAndTheRoutineContinues() {
        val h = Harness()
        var next = false
        val routine = sequential(race(h.drive.followCommand(h.line()), waitMs(30.0)), instant { next = true })
        Scheduler.schedule(routine)
        h.tick()
        assertTrue(h.hardware.powers().any { it != 0.0 })
        // Ivy's waitMs runs on wall-clock time.
        Thread.sleep(50)
        repeat(3) { h.tick() }
        assertTrue(next)
        assertEquals("IDLE", h.drive.driveModeName)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun holdHoldsTheRequestedPoseIncludingHeadingAndWaitsForAnUpdate() {
        val h = Harness()
        val target = Pose(10.0, 20.0, 1.5)
        val hold = h.drive.holdCommand(target)
        Scheduler.schedule(hold)
        assertEquals("HOLDING", h.drive.driveModeName)
        val held = h.drive.follower.poseAt(0.0)
        assertEquals(10.0, held.x(), 1e-9)
        assertEquals(20.0, held.y(), 1e-9)
        assertEquals(1.5, held.heading(), 1e-9)

        // Already there, but no update has measured it since the command started.
        h.place(target)
        assertFalse(hold.done())
        // This tick's scheduler pass still precedes the update that measures arrival.
        h.tick()
        assertTrue(Scheduler.isScheduled(hold))
        h.tick()
        assertFalse(Scheduler.isScheduled(hold))
    }

    @Test
    fun holdTimeoutIsABoundedWaitAndTheFollowerKeepsHolding() {
        val h = Harness()
        val hold = h.drive.holdCommand(Pose(24.0, 0.0, 0.0), timeoutMs = 500.0)
        Scheduler.schedule(hold)
        h.tick()
        assertTrue(Scheduler.isScheduled(hold))
        h.tick(600.0)
        assertFalse(Scheduler.isScheduled(hold))
        assertEquals(0, h.robot.commandFaultCount)
        assertEquals("HOLDING", h.drive.driveModeName)
    }

    @Test
    fun unfinishedTurnDoesNotAdvanceItsSequenceAndCompletesOnMeasuredHeading() {
        val h = Harness()
        var advanced = false
        val sequence = sequential(h.drive.turnToCommand(PI / 2), instant { advanced = true })
        Scheduler.schedule(sequence)
        repeat(5) { h.tick() }
        assertFalse(advanced)
        assertTrue(Scheduler.isScheduled(sequence))
        assertTrue(h.hardware.powers().any { it != 0.0 })

        h.place(Pose(0.0, 0.0, PI / 2))
        repeat(3) { h.tick() }
        assertTrue(advanced)
        assertFalse(Scheduler.isScheduled(sequence))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun turnTimeoutAbortsTheRoutineInsteadOfRunningItsNextStep() {
        val h = Harness()
        var advanced = false
        val sequence: Command = sequential(h.drive.turnToCommand(PI / 2, timeoutMs = 250.0), instant { advanced = true })
        Scheduler.schedule(sequence)
        h.tick()
        h.tick(250.0)

        assertEquals(1, h.robot.commandFaultCount)
        assertNotNull(h.robot.lastCommandFault!!.message!!.let { if ("turnTo timed out" in it) it else null })
        assertFalse(advanced)
        assertFalse(Scheduler.isScheduled(sequence))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        h.tick()
        assertFalse(advanced)
    }

    @Test
    fun wraparoundHeadingUsesTheShortestAngleAndRequiresARealUpdate() {
        val h = Harness()
        h.place(Pose(0.0, 0.0, Math.toRadians(359.5)))
        val turn = h.drive.turnToCommand(0.0)
        Scheduler.schedule(turn)
        assertFalse(turn.done())
        h.tick()
        assertTrue(Scheduler.isScheduled(turn))
        h.tick()
        assertFalse(Scheduler.isScheduled(turn))
    }

    @Test
    fun cancellingATurnStopsTheHoldController() {
        val h = Harness()
        val turn = h.drive.turnToCommand(PI / 2)
        Scheduler.schedule(turn)
        h.tick()
        Scheduler.cancel(turn)
        h.tick()
        assertEquals("IDLE", h.drive.driveModeName)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun invalidTargetsAndTimeoutsAreRejected() {
        val drive = Harness().drive
        assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(Double.NaN) }
        assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(Double.POSITIVE_INFINITY) }
        for (timeout in listOf(-1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(0.0, timeout) }
            assertThrows(IllegalArgumentException::class.java) { drive.holdCommand(Pose.zero(), timeout) }
        }
    }
}
