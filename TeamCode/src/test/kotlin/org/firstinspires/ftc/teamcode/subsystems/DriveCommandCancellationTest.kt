package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.ivy.commands.Commands.lazy
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.TeleopInput
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Ivy 1.1.1 ends group children that never started and can end a child
 * twice (pinned in LibraryContractTest). A drive command must never drive
 * after it ends, and its cleanup must never energize the drivetrain.
 */
class DriveCommandCancellationTest {

    private val hardware = PedroDriveFixture()
    private val drive = hardware.drive
    private val robot = Robot(hardware.hardwareMap, hardware.clock).also {
        it.register(drive)
        it.start()
    }
    private val executions = mutableMapOf<String, Int>()

    private fun tick() {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    /** Full-forward sticks; counts the ticks it actually drives. */
    private fun assist(name: String = "assist") = drive.teleopCommand {
        executions[name] = (executions[name] ?: 0) + 1
        TeleopInput(1.0, 0.0, 0.0)
    }

    @Test
    fun cancellingASequenceEndsItsUnstartedDriveStepsWithoutEffect() {
        val group = sequential(
            drive.followCommand(Paths.line(Pose(0.0, 0.0), Pose(48.0, 0.0)).constant(0.0)),
            drive.holdCommand(Pose(48.0, 0.0, 0.0)),
            drive.turnToCommand(Math.PI / 2),
            assist(),
        )
        Scheduler.schedule(group)
        tick()
        assertEquals("FOLLOWING", drive.driveModeName)
        assertTrue(hardware.powers().any { it != 0.0 })

        Scheduler.cancel(group)
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        repeat(3) { tick() }

        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
        assertEquals("IDLE", drive.driveModeName)
        // The assist never started, so it never drove.
        assertTrue(executions.isEmpty())
    }

    @Test
    fun aDeadlineChildIsCleanedUpOnceAndNotReportedAsANaturalEnd() {
        var deadlineDone = false
        val group = deadline(waitUntil { deadlineDone }, assist())
        Scheduler.schedule(group)
        tick()
        assertTrue(hardware.powers().all { it > 0.0 })

        deadlineDone = true
        tick()
        val driven = executions.getValue("assist")
        repeat(3) { tick() }

        assertFalse(Scheduler.isScheduled(group))
        assertEquals(driven, executions.getValue("assist"))
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun anAssistInterruptedEarlierInTheSameTickDoesNotStageDriving() {
        var takeOver = false
        val override = Command.build().requiring(drive)
        // Runs before the assist in Ivy's order and interrupts it from inside execute.
        Scheduler.schedule(infinite { if (takeOver) Scheduler.schedule(override) })
        val victim = assist()
        Scheduler.schedule(victim)
        tick()
        assertTrue(hardware.powers().all { it > 0.0 })

        takeOver = true
        tick()

        assertFalse(Scheduler.isScheduled(victim))
        assertTrue(Scheduler.isScheduled(override))
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun aLazyEndedBeforeStartDoesNotCleanUpItsPreviousCommandAgain() {
        val stepped = lazy { assist("lazy") }.requiring(drive)
        Scheduler.schedule(stepped)
        tick()
        Scheduler.cancel(stepped)
        assertEquals(1, executions.getValue("lazy"))

        // Same lazy, reused as a later step that is never reached.
        val group = sequential(waitUntil { false }, stepped)
        Scheduler.schedule(group)
        tick()
        Scheduler.cancel(group)
        tick()

        assertEquals(1, executions.getValue("lazy"))
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun repeatedEndsAfterARealRunDoNothing() {
        val hold = drive.holdCommand(Pose(0.0, 0.0, 0.0), timeoutMs = 10_000.0)
        Scheduler.schedule(hold)
        tick()
        assertEquals("HOLDING", drive.driveModeName)
        Scheduler.cancel(hold)
        assertEquals("IDLE", drive.driveModeName)

        // A later hold owns the follower; the old instance ending again must not stop it.
        val next = drive.holdCommand(Pose(0.0, 0.0, 0.0), timeoutMs = 10_000.0)
        Scheduler.schedule(next)
        tick()
        hold.end(com.pedropathing.ivy.behaviors.EndCondition.INTERRUPTED)
        assertEquals("HOLDING", drive.driveModeName)
        assertTrue(Scheduler.isScheduled(next))
    }
}
