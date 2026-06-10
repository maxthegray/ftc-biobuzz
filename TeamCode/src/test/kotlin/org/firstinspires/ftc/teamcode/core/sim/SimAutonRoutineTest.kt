package org.firstinspires.ftc.teamcode.core.sim

import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.pathing.autoRoutine
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.util.Alliance
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Full auton routines executed headless: real PathDSL geometry, real Ivy
 * scheduling, real subsystem lifecycle — only the follower's control law is
 * simulated. This is the test layer that catches routine-logic bugs
 * (sequencing, mirroring, mode transitions, handoff) without carpet time.
 */
class SimAutonRoutineTest {

    private val startRed = Pose(8.0, 56.0, 0.0)
    private val outRed = Pose(32.0, 56.0, 0.0)

    @Before
    fun setUp() {
        Scheduler.reset()
        PersistedPose.storageFile = null
        PersistedPose.clear()
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        PersistedPose.clear()
    }

    /** The ExampleAuto shape, minus wall-clock waits (waitMs is real-time). */
    private fun buildRoutine(
        sim: SimHarness,
        alliance: Alliance,
        onEvent: ((String) -> Unit)? = null,
    ): PedroAutoRunner {
        val outPath = sim.drive.path(startPose = startRed, alliance = alliance) {
            lineTo(outRed)
            constantHeading(0.0)
        }
        val backPath = sim.drive.path(startPose = outRed, alliance = alliance) {
            lineTo(startRed)
            linearHeading(Math.toRadians(90.0), 0.0)
        }
        sim.follower.setStartingPose(alliance.mirror(startRed))
        return autoRoutine(sim.drive, onEvent) {
            follow(outPath)
            holdPose(alliance.mirror(outRed))
            turnTo(alliance.mirror(Math.toRadians(90.0)))
            followAndHold(backPath)
        }
    }

    @Test
    fun redRoutineRunsToCompletionAndEndsAtStart() {
        val sim = SimHarness()
        sim.start()
        val runner = buildRoutine(sim, Alliance.RED)
        runner.schedule()

        assertTrue("routine did not finish in sim time", sim.runUntil(15.0) { runner.isDone })

        // Back path ends where the routine started, heading interpolated to 0.
        assertPose(startRed, sim.drive.pose)
        // followAndHold leaves the drive holding the end pose.
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, sim.drive.mode)
        assertFalse(sim.follower.isBusy)
    }

    @Test
    fun blueRoutineEndsAtTheMirroredPose() {
        val sim = SimHarness()
        sim.start()
        val runner = buildRoutine(sim, Alliance.BLUE)
        runner.schedule()

        assertTrue(sim.runUntil(15.0) { runner.isDone })

        assertPose(Alliance.BLUE.mirror(startRed), sim.drive.pose)
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, sim.drive.mode)
    }

    @Test
    fun stepEventsTellTheRoutineTimeline() {
        val sim = SimHarness()
        sim.start()
        val events = mutableListOf<String>()
        val runner = buildRoutine(sim, Alliance.RED) { events += it }
        runner.schedule()

        assertTrue(sim.runUntil(15.0) { runner.isDone })

        assertEquals(5, events.size)
        assertEquals("auto step 1/4: follow", events[0])
        assertEquals("auto step 2/4: holdPose", events[1])
        assertTrue(events[2].startsWith("auto step 3/4: turnTo"))
        assertEquals("auto step 4/4: followAndHold", events[3])
        assertEquals("auto routine complete", events[4])
    }

    @Test
    fun cancellingMidPathIdlesTheDrive() {
        val sim = SimHarness()
        sim.start()
        val runner = buildRoutine(sim, Alliance.RED)
        runner.schedule()

        assertTrue(sim.runUntil(5.0) { sim.drive.mode == MecanumDriveSubsystem.Mode.FOLLOWING })
        val breaksBefore = sim.follower.breakFollowingCalls
        runner.cancel()
        sim.tick()

        assertTrue(runner.isDone)
        assertEquals(MecanumDriveSubsystem.Mode.IDLE, sim.drive.mode)
        assertTrue(sim.follower.breakFollowingCalls > breaksBefore)
        assertFalse(sim.follower.isBusy)
    }

    @Test
    fun finalPoseHandsOffToTheNextOpMode() {
        val sim = SimHarness()
        sim.start()
        val runner = buildRoutine(sim, Alliance.RED)
        runner.schedule()
        assertTrue(sim.runUntil(15.0) { runner.isDone })
        sim.robot.stop()

        // "Teleop" boots fresh: new harness, pose restored from the handoff.
        val teleop = SimHarness()
        teleop.start()
        assertTrue(teleop.localizer.restorePersistedPose())
        assertPose(startRed, teleop.drive.pose)
    }

    private fun assertPose(expected: Pose, actual: Pose, tolerance: Double = 1e-3) {
        assertEquals("x", expected.x, actual.x, tolerance)
        assertEquals("y", expected.y, actual.y, tolerance)
        assertEquals(
            "heading",
            0.0,
            com.pedropathing.math.MathFunctions.normalizeAngleSigned(expected.heading - actual.heading),
            tolerance,
        )
    }
}
