package org.firstinspires.ftc.teamcode.core.pathing

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.fakeFollower
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.fail
import org.junit.Assert.assertTrue
import org.junit.Test

class PedroAutoRunnerTest {

    private val clock = FakeClock()
    private val robot = Robot(HardwareMap(null, null), clock)
    private val drive = robot.register(MecanumDriveSubsystem(fakeFollower()))

    @Test
    fun deadlineCombinesChildRequirementsAndUsesAutonPriority() {
        val firstReq = Any()
        val secondReq = Any()

        val command = PedroAutoRunner(robot, drive)
            .deadline {
                then(commandWithRequirement(firstReq))
                then(commandWithRequirement(secondReq))
            }
            .build()

        assertTrue(command.requirements().contains(firstReq))
        assertTrue(command.requirements().contains(secondReq))
        assertEquals(CommandPriorities.AUTON_ROUTINE, command.priority())
    }

    @Test
    fun timeoutWrapsWholeRoutineWithoutDroppingRequirements() {
        val requirement = Any()

        val command = PedroAutoRunner(robot, drive)
            .then(commandWithRequirement(requirement))
            .timeout(100.0)
            .build()

        assertTrue(command.requirements().contains(requirement))
        assertEquals(CommandPriorities.AUTON_ROUTINE, command.priority())
    }

    @Test
    fun groupRejectsSharedRequirements() {
        val requirement = Any()

        try {
            PedroAutoRunner(robot, drive).parallel {
                then(commandWithRequirement(requirement))
                then(commandWithRequirement(requirement))
            }
            fail("shared requirements should be rejected")
        } catch (_: IllegalArgumentException) {
            // expected
        }
    }

    @Test
    fun timeoutCancelsTheRoutineInVirtualTime() {
        val runner = PedroAutoRunner(robot, drive)
            .then(commandWithRequirement(Any())) // never finishes on its own
            .timeout(100.0)

        runner.schedule()
        assertFalse(runner.isDone)

        clock.advanceMs(50.0)
        robot.scheduler.execute()
        assertFalse(runner.isDone)

        clock.advanceMs(60.0)
        robot.scheduler.execute()
        assertTrue(runner.isDone)
    }

    @Test
    fun waitStepsRunOnTheRobotClock() {
        var ran = false
        val runner = PedroAutoRunner(robot, drive)
            .wait(200)
            .run { ran = true }

        runner.schedule()
        repeat(3) { robot.scheduler.execute() }
        assertFalse(ran)

        clock.advanceMs(250.0)
        // The wait completes and the following instant drains in the same tick.
        robot.scheduler.execute()
        assertTrue(ran)
    }

    @Test
    fun stepTimeoutInterruptsTheStepAndTheRoutineContinues() {
        val ends = mutableListOf<EndCondition>()
        val stuck = CommandBuilder()
            .requiring(Any())
            .setDone { false } // never finishes on its own
            .setEnd { ends += it }
        var afterRan = false
        val runner = PedroAutoRunner(robot, drive)
            .timeout(100) { then(stuck) }
            .run { afterRan = true }

        runner.schedule()
        robot.scheduler.execute()
        assertFalse(afterRan)
        assertTrue(ends.isEmpty())

        clock.advanceMs(150.0)
        robot.scheduler.execute()
        assertEquals(listOf(EndCondition.INTERRUPTED), ends)
        assertTrue(afterRan)
        assertTrue(runner.isDone)
    }

    private fun commandWithRequirement(requirement: Any): Command =
        CommandBuilder().requiring(requirement).setDone { false }
}
