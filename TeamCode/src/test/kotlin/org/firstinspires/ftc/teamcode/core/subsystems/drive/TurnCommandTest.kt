package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathConstraints
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.firstinspires.ftc.teamcode.core.command.Groups
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.PI

class TurnCommandTest {
    private class Harness {
        val hardware = PedroDriveFixture(PathConstraints(0.99, 100.0))
        val clock = FakeClock()
        val drive = MecanumDriveSubsystem(hardware.follower, clock)
        val robot = Robot(hardware.hardwareMap, clock)
        init { robot.register(drive); robot.start() }
        fun tick(ms: Double = 20.0) { clock.advanceMs(ms); robot.loop() }
    }

    @Test
    fun pedrosPointPathTimeoutCannotAdvanceAnUnfinishedTurn() {
        val h = Harness()
        var advanced = false
        val sequence = Groups.sequential(
            h.drive.turnToCommand(PI / 2),
            Commands.instant { advanced = true },
        )
        h.robot.scheduler.schedule(sequence)
        h.tick()
        // Pedro's point-path timer uses wall time, not the injected test clock.
        Thread.sleep(150)
        h.tick(150.0)
        h.tick()

        assertFalse(advanced)
        assertTrue(h.robot.scheduler.isScheduled(sequence))
        assertTrue(h.hardware.powers().any { it != 0.0 })

        h.hardware.localizer.measuredPose = Pose(0.0, 0.0, PI / 2)
        h.tick() // measure the new heading
        h.tick() // complete from the measured heading
        assertTrue(advanced)
        assertFalse(h.robot.scheduler.isScheduled(sequence))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun timeoutFaultsTheWholeSequenceInsteadOfRunningItsNextAction() {
        val h = Harness()
        var advanced = false
        val sequence = Groups.sequential(
            h.drive.turnToCommand(PI / 2, timeoutMs = 250.0),
            Commands.instant { advanced = true },
        )
        h.robot.scheduler.schedule(sequence)
        h.tick()
        val failure = assertThrows(IllegalStateException::class.java) { h.tick(250.0) }
        assertTrue(failure.message!!.contains("turnTo timed out"))
        assertFalse(advanced)
        assertFalse(h.robot.scheduler.isScheduled(sequence))
        assertEquals(MecanumDriveSubsystem.Mode.IDLE, h.drive.mode)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun wraparoundHeadingIsComparedByShortestAngleAndRequiresARealUpdate() {
        val h = Harness()
        h.hardware.localizer.measuredPose = Pose(0.0, 0.0, Math.toRadians(359.5))
        val turn = h.drive.turnToCommand(0.0)
        h.robot.scheduler.schedule(turn)
        h.tick()
        assertTrue(h.robot.scheduler.isScheduled(turn))
        h.tick()
        assertFalse(h.robot.scheduler.isScheduled(turn))
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun cancellingATurnStopsTheHoldController() {
        val h = Harness()
        val turn = h.drive.turnToCommand(PI / 2)
        h.robot.scheduler.schedule(turn)
        h.tick()
        h.robot.scheduler.cancel(turn)
        h.tick()
        assertEquals(MecanumDriveSubsystem.Mode.IDLE, h.drive.mode)
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
    }

    @Test
    fun invalidTargetsAndTimeoutsAreRejected() {
        val drive = Harness().drive
        assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(Double.NaN) }
        assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(Double.POSITIVE_INFINITY) }
        for (timeout in listOf(-1.0, Double.NaN, Double.POSITIVE_INFINITY)) {
            assertThrows(IllegalArgumentException::class.java) { drive.turnToCommand(0.0, timeout) }
        }
    }
}
