package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.fakeFollower
import org.junit.Assert.assertEquals
import org.junit.Assert.fail
import org.junit.Assert.assertTrue
import org.junit.Test

class PedroAutoRunnerTest {

    private val drive = MecanumDriveSubsystem(fakeFollower())

    @Test
    fun deadlineCombinesChildRequirementsAndUsesAutonPriority() {
        val firstReq = Any()
        val secondReq = Any()

        val command = PedroAutoRunner(drive)
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

        val command = PedroAutoRunner(drive)
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
            PedroAutoRunner(drive).parallel {
                then(commandWithRequirement(requirement))
                then(commandWithRequirement(requirement))
            }
            fail("shared requirements should be rejected")
        } catch (_: IllegalArgumentException) {
            // expected
        }
    }

    private fun commandWithRequirement(requirement: Any): Command =
        CommandBuilder().requiring(requirement).setDone { false }
}
