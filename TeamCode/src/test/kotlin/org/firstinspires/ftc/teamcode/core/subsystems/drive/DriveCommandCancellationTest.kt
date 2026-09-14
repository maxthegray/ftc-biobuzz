package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.ivy.commands.Commands.lazy
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Ivy 1.1.1 ends group children that never started and can end a child
 * twice (pinned in LibraryContractTest). Drive cleanup must run once per
 * start and never energize the drivetrain.
 */
class DriveCommandCancellationTest {

    private val hardware = PedroDriveFixture()
    private val drive = hardware.drive
    private val robot = Robot(hardware.hardwareMap, hardware.clock).also {
        it.register(drive)
        it.start()
    }
    private val events = mutableListOf<String>()

    private fun tick() {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    private fun assist(name: String = "assist") = drive.teleopCommand(
        priority = CommandPriorities.AUTON_ROUTINE,
        onStart = { events += "$name start" },
        onEnd = { events += "$name end $it" },
    ) { TeleopInput(1.0, 0.0, 0.0) }

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
        // The assist never started, so its cleanup callback never ran.
        assertTrue(events.isEmpty())
    }

    @Test
    fun aDeadlineChildIsCleanedUpOnceAndNotReportedAsANaturalEnd() {
        var deadlineDone = false
        val group = deadline(waitUntil { deadlineDone }, assist())
        Scheduler.schedule(group)
        tick()
        assertTrue(hardware.powers().all { it > 0.0 })

        deadlineDone = true
        repeat(3) { tick() }

        assertFalse(Scheduler.isScheduled(group))
        assertEquals(listOf("assist start", "assist end INTERRUPTED"), events)
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun anAssistInterruptedEarlierInTheSameTickDoesNotStageDriving() {
        var takeOver = false
        val override = Command.build().requiring(drive).setPriority(CommandPriorities.DRIVER_OVERRIDE)
        // Runs before the assist in Ivy's order and interrupts it from inside execute.
        Scheduler.schedule(infinite { if (takeOver) Scheduler.schedule(override) })
        val victim = assist()
        Scheduler.schedule(victim)
        tick()
        assertTrue(hardware.powers().all { it > 0.0 })

        takeOver = true
        tick()

        assertEquals(listOf("assist start", "assist end INTERRUPTED"), events)
        assertTrue(Scheduler.isScheduled(override))
        assertArrayEquals(DoubleArray(4), hardware.powers(), 0.0)
    }

    @Test
    fun aLazyEndedBeforeStartDoesNotCleanUpItsPreviousCommandAgain() {
        val stepped = lazy { assist("lazy") }.requiring(drive).setPriority(CommandPriorities.AUTON_ROUTINE)
        Scheduler.schedule(stepped)
        tick()
        Scheduler.cancel(stepped)
        assertEquals(listOf("lazy start", "lazy end INTERRUPTED"), events)

        // Same lazy, reused as a later step that is never reached.
        val group = sequential(waitUntil { false }, stepped)
        Scheduler.schedule(group)
        tick()
        Scheduler.cancel(group)
        tick()

        assertEquals(listOf("lazy start", "lazy end INTERRUPTED"), events)
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
