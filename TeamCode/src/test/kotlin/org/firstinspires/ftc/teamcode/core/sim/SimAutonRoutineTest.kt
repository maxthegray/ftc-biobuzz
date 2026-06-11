package org.firstinspires.ftc.teamcode.core.sim

import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.normalizeAngleSigned
import org.firstinspires.ftc.teamcode.core.pathing.PedroAutoRunner
import org.firstinspires.ftc.teamcode.core.pathing.toPedro
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
 * Full auton routines executed headless: real PathDSL geometry, real command
 * scheduling, real subsystem lifecycle — only the follower's control law is
 * simulated. This is the test layer that catches routine-logic bugs
 * (sequencing, mirroring, mode transitions, handoff) without carpet time.
 * Waits run on the harness clock, so the routine matches ExampleAuto exactly.
 */
class SimAutonRoutineTest {

    private val startRed = Pose2d(8.0, 56.0, 0.0)
    private val outRed = Pose2d(32.0, 56.0, 0.0)

    @Before
    fun setUp() {
        PersistedPose.storageFile = null
        PersistedPose.clear()
    }

    @After
    fun tearDown() {
        PersistedPose.clear()
    }

    /** The ExampleAuto shape, including its virtual-time wait. */
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
        sim.follower.setStartingPose(alliance.mirror(startRed).toPedro())
        return autoRoutine(sim.robot, sim.drive, onEvent) {
            follow(outPath)
            holdPose(alliance.mirror(outRed))
            wait(300)
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

        assertEquals(6, events.size)
        assertEquals("auto step 1/5: follow", events[0])
        assertEquals("auto step 2/5: holdPose", events[1])
        assertEquals("auto step 3/5: wait 300 ms", events[2])
        assertTrue(events[3].startsWith("auto step 4/5: turnTo"))
        assertEquals("auto step 5/5: followAndHold", events[4])
        assertEquals("auto routine complete", events[5])
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

    @Test
    fun progressMarkersFireMidPath() {
        val sim = SimHarness()
        sim.start()
        val outPath = sim.drive.path(startPose = startRed, alliance = Alliance.RED) {
            lineTo(outRed)
            constantHeading(0.0)
        }
        sim.follower.setStartingPose(startRed.toPedro())
        var fires = 0
        var markerPose: Pose2d? = null
        val events = mutableListOf<String>()
        val runner = autoRoutine(sim.robot, sim.drive, { events += it }) {
            follow(outPath) {
                at(0.5, "halfway") {
                    fires++
                    markerPose = sim.drive.pose
                }
            }
        }
        runner.schedule()

        assertTrue(sim.runUntil(10.0) { runner.isDone })

        assertEquals(1, fires)
        val x = markerPose!!.x
        assertTrue(
            "marker should fire mid-path, fired at x=$x",
            x > startRed.x + 6.0 && x < outRed.x - 4.0,
        )
        assertTrue(events.any { it.startsWith("halfway @ 50%") })
        assertTrue(events.any { "follow+markers(1)" in it })
        // The drive still finishes the path normally.
        assertPose(outRed, sim.drive.pose, tolerance = 1.0)
    }

    @Test
    fun unreachedMarkersAreDroppedWhenTheRoutineIsCancelled() {
        val sim = SimHarness()
        sim.start()
        val outPath = sim.drive.path(startPose = startRed, alliance = Alliance.RED) {
            lineTo(outRed)
            constantHeading(0.0)
        }
        sim.follower.setStartingPose(startRed.toPedro())
        var fired = false
        val runner = autoRoutine(sim.robot, sim.drive) {
            follow(outPath) {
                at(0.95, "late") { fired = true }
            }
        }
        runner.schedule()

        assertTrue(sim.runUntil(5.0) { sim.drive.mode == MecanumDriveSubsystem.Mode.FOLLOWING })
        repeat(3) { sim.tick() }
        runner.cancel()
        sim.tick()

        assertTrue(runner.isDone)
        assertFalse("a marker the path never reached must not fire", fired)
        assertEquals(MecanumDriveSubsystem.Mode.IDLE, sim.drive.mode)
    }

    private fun assertPose(expected: Pose2d, actual: Pose2d, tolerance: Double = 1e-3) {
        assertEquals("x", expected.x, actual.x, tolerance)
        assertEquals("y", expected.y, actual.y, tolerance)
        assertEquals(
            "heading",
            0.0,
            normalizeAngleSigned(expected.heading - actual.heading),
            tolerance,
        )
    }
}
