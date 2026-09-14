package org.firstinspires.ftc.teamcode.core

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.BlockedBehavior
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.commands.Commands.infinite
import com.pedropathing.ivy.commands.Commands.lazy
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Behaviour of the published Ivy 1.1.1 and Pedro Pathing 3.0.0 artifacts that
 * this repo's design depends on or works around. A library upgrade that
 * changes any of these fails here, naming the assumption to revisit.
 */
class LibraryContractTest {

    @Before
    fun resetScheduler() = Scheduler.reset()

    @Test
    fun ivyResetDropsCommandsWithoutRunningEndHandlers() {
        // Why Robot.stop() stops subsystems itself and onCommandFault() exists.
        val ends = mutableListOf<EndCondition>()
        val command = infinite {}.setEnd { ends += it }
        Scheduler.schedule(command)
        Scheduler.reset()
        assertFalse(Scheduler.isScheduled(command))
        assertTrue(ends.isEmpty())
    }

    @Test
    fun ivyCancelCascadesInterruptedIntoGroupChildrenIncludingOnesThatNeverStarted() {
        // Why every end handler must tolerate an end without a start.
        val log = mutableListOf<String>()
        val group = sequential(
            infinite {}.setStart { log += "a:start" }.setEnd { log += "a:$it" },
            infinite {}.setStart { log += "b:start" }.setEnd { log += "b:$it" },
        )
        Scheduler.schedule(group)
        Scheduler.execute()
        Scheduler.cancel(group)
        assertEquals(listOf("a:start", "a:INTERRUPTED", "b:INTERRUPTED"), log)
    }

    @Test
    fun ivyCancelEndsAQueuedCommandThatNeverStarted() {
        val requirement = Any()
        val log = mutableListOf<String>()
        Scheduler.schedule(infinite {}.requiring(requirement).setPriority(5))
        val queued = infinite {}.requiring(requirement)
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .setStart { log += "start" }
            .setEnd { log += "end:$it" }
        Scheduler.schedule(queued)
        assertTrue(Scheduler.isScheduled(queued))
        Scheduler.cancel(queued)
        assertEquals(listOf("end:INTERRUPTED"), log)
    }

    @Test
    fun ivyLazyEndedWithoutStartingForwardsToItsPreviousCommand() {
        val log = mutableListOf<String>()
        var run = 0
        val stepped = lazy {
            val n = ++run
            infinite {}.setEnd { log += "inner$n:$it" }
        }
        Scheduler.schedule(stepped)
        Scheduler.cancel(stepped)
        stepped.end(EndCondition.INTERRUPTED)
        assertEquals(listOf("inner1:INTERRUPTED", "inner1:INTERRUPTED"), log)
    }

    @Test
    fun ivyConflictCancelKeepsADefaultFromPreemptingEqualPriority() {
        val requirement = Any()
        val explicit = infinite {}.requiring(requirement)
        val default = infinite {}.requiring(requirement).setConflictBehavior(ConflictBehavior.CANCEL)
        Scheduler.schedule(explicit)
        Scheduler.schedule(default)
        assertTrue(Scheduler.isScheduled(explicit))
        assertFalse(Scheduler.isScheduled(default))
    }

    @Test
    fun ivyDeadlineEndsAnUnfinishedChildTwice() {
        // Drive command end handlers must be idempotent and must not treat a
        // trailing NATURALLY as success.
        var deadlineDone = false
        val ends = mutableListOf<String>()
        Scheduler.schedule(
            deadline(
                Command.build().setDone { deadlineDone },
                waitUntil { false }.setEnd { ends += it.name },
            ),
        )
        Scheduler.execute()
        deadlineDone = true
        Scheduler.execute()
        Scheduler.execute()
        assertEquals(listOf("INTERRUPTED", "NATURALLY"), ends)
    }

    @Test
    fun ivyStillExecutesACommandInterruptedFromInsideExecuteThatTick() {
        // Why bindings, defaults and the localizer fault policy schedule
        // outside Scheduler.execute().
        val requirement = Any()
        val log = mutableListOf<String>()
        val scheduler = Command.build()
            .setExecute { Scheduler.schedule(infinite {}.requiring(requirement).setPriority(5)) }
            .setDone { true }
        val victim = Command.build().requiring(requirement).setExecute { log += "executed" }.setEnd { log += "end:$it" }
        Scheduler.schedule(scheduler)
        Scheduler.schedule(victim)
        Scheduler.execute()
        assertEquals(listOf("end:INTERRUPTED", "executed"), log)
    }

    @Test
    fun pedroLinearHeadingRunsBackwardsOnLinesButNotOnCurves() {
        val a = Pose(0.0, 0.0, 0.0)
        val b = Pose(48.0, 0.0, Math.PI / 2)
        val line = Paths.line(a, b).linear(a, b)
        assertEquals(Math.PI / 2, line.heading(0.0), 1e-9)
        assertEquals(0.0, line.heading(1.0), 1e-9)

        val curve = Paths.curve(a, Pose(24.0, 0.0), b).linear(a, b)
        assertEquals(0.0, curve.heading(0.0), 1e-9)
        assertEquals(Math.PI / 2, curve.heading(1.0), 1e-9)

        assertThrows(IllegalArgumentException::class.java) { Paths.curve(a, b) }
    }

    @Test
    fun pedroPoseFactoryMirrorXIsNotTheFieldReflection() {
        // Why Alliance.poses() maps with Alliance.mirror instead.
        val mirrored = PoseFactory.radians().mirrorX(70.75).of(10.0, 20.0, 0.3)
        assertEquals(131.5, mirrored.x(), 1e-9)
        assertEquals(2 * Math.PI - 0.3, mirrored.heading(), 1e-9)
    }

    @Test
    fun pedroPosesAreImmutableAndNormalizeHeading() {
        val pose = Pose(1.0, 2.0, -0.1)
        pose.withHeading(1.0)
        assertEquals(2 * Math.PI - 0.1, pose.heading(), 1e-12)
    }

    @Test
    fun pedroLeavesFollowModeAtTheParametricEndAndIsBusyIsNoCompletionSignal() {
        val hardware = PedroDriveFixture()
        val follower = hardware.follower
        follower.holdEnd.set(false)
        hardware.localizer.measuredPose = Pose(0.0, 0.0, 0.0)
        follower.follow(Paths.line(Pose(0.0, 0.0), Pose(24.0, 0.0)).constant(0.0))
        follower.update(0.02)
        hardware.localizer.measuredPose = Pose(24.0, 3.0, 0.0)
        follower.update(0.02)
        follower.update(0.02)
        assertEquals(Follower.Mode.IDLE, follower.mode())
        assertTrue(follower.isBusy)
    }

    @Test
    fun pedroStopOnlyChangesModeUntilTheNextUpdate() {
        // Why MecanumDriveSubsystem.halt() also stops the drivetrain directly.
        val hardware = PedroDriveFixture()
        val follower = hardware.follower
        follower.manual(1.0, 0.0, 0.0)
        follower.update(0.02)
        follower.stop()
        assertTrue(hardware.powers().all { it == 1.0 })
        follower.update(0.02)
        assertTrue(hardware.powers().all { it == 0.0 })
    }
}
