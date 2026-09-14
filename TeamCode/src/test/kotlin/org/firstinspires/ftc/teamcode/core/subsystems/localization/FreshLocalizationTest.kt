package org.firstinspires.ftc.teamcode.core.subsystems.localization

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.drive.PedroDriveFixture
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Every op-mode starts from its own start pose, whatever the Pinpoint still
 * holds from the previous run. One [PedroDriveFixture.OdometryProbe] is shared
 * between runs to model the same device across op-modes.
 */
class FreshLocalizationTest {

    private class Run(
        odometry: PedroDriveFixture.OdometryProbe,
        startingPose: Pose = Pose.zero(),
    ) {
        val hardware = PedroDriveFixture(localizer = odometry)
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        val drive = robot.register(hardware.drive)
        val localizer = robot.register(
            LocalizerSubsystem(hardware.follower, hardware.clock, isFollowing = drive::isFollowing, startingPose = startingPose),
        )
        var input = TeleopInput(0.0, 0.0, 0.0)

        init {
            drive.defaultCommand = drive.teleopCommand { input }
            hardware.clearWrites()
        }

        /**
         * INIT pressed, then [ticks] init-loop iterations. Returns, per tick, the
         * pose the robot code sees and whether the localizer reports ready.
         */
        fun init(ticks: Int = 1): List<Pair<Pose, Boolean>> {
            robot.init()
            return List(ticks) {
                hardware.clock.advanceMs(20.0)
                robot.initTick()
                drive.pose to localizer.ready
            }
        }

        fun startAndDrive(to: Pose, ticks: Int = 3) {
            robot.start()
            input = TeleopInput(0.5, 0.0, 0.0)
            repeat(ticks) {
                hardware.clock.advanceMs(20.0)
                robot.loop()
            }
            hardware.localizer.measuredPose = to
            hardware.clock.advanceMs(20.0)
            robot.loop()
        }
    }

    private fun assertPose(expected: Pose, actual: Pose) =
        assertArrayEquals(
            doubleArrayOf(expected.x(), expected.y(), expected.heading()),
            doubleArrayOf(actual.x(), actual.y(), actual.heading()),
            1e-9,
        )

    @Test
    fun consecutiveTeleopRunsEachStartAtZero() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        val first = Run(pinpoint)
        first.init()
        first.startAndDrive(to = Pose(30.0, 10.0, 1.2))
        first.robot.stop()
        assertPose(Pose(30.0, 10.0, 1.2), pinpoint.devicePose)

        // The first read after the reset is a sample from before it.
        pinpoint.staleReadsAfterSetPose = 1
        val second = Run(pinpoint)
        val writesBefore = pinpoint.setPoseCalls.size
        val ticks = second.init(ticks = 3)
        // The stale read is rejected and the reset re-written before anything sees it.
        ticks.forEach { assertPose(Pose.zero(), it.first) }
        assertEquals(listOf(false, true, true), ticks.map { it.second })
        // One write at INIT, one re-write after the stale read.
        assertEquals(2, pinpoint.setPoseCalls.size - writesBefore)
        assertTrue(pinpoint.setPoseCalls.takeLast(2).all { it.x() == 0.0 && it.y() == 0.0 && it.heading() == 0.0 })
        // INIT reads and writes the pose; it never commands the motors.
        assertTrue(second.hardware.motors.all { it.writes.isEmpty() })

        second.robot.start()
        second.robot.loop()
        assertPose(Pose.zero(), second.drive.pose)
        assertArrayEquals(DoubleArray(4), second.hardware.powers(), 0.0)
    }

    @Test
    fun teleopAfterAutonomousDoesNotInheritTheAutonomousPose() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        val auto = Run(pinpoint, startingPose = Pose(8.0, 56.0, Math.PI / 2))
        auto.init()
        assertPose(Pose(8.0, 56.0, Math.PI / 2), auto.drive.pose)
        auto.startAndDrive(to = Pose(40.0, 60.0, Math.PI))
        auto.robot.stop()

        val teleop = Run(pinpoint)
        teleop.init()
        assertTrue(teleop.localizer.ready)
        assertPose(Pose.zero(), teleop.drive.pose)
        assertPose(Pose.zero(), pinpoint.devicePose)
    }

    @Test
    fun poseLeftByACrashedRunIsNeverReportedAsReady() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        val crashed = Run(pinpoint)
        crashed.init()
        crashed.startAndDrive(to = Pose(50.0, -20.0, 2.0))
        // No stop(): the previous op-mode died with the device holding its pose,
        // and it keeps returning old samples for a while after the next write.
        pinpoint.staleReadsAfterSetPose = 3

        val next = Run(pinpoint)
        next.robot.init()
        repeat(3) {
            next.hardware.clock.advanceMs(20.0)
            next.robot.initTick()
            assertFalse(next.localizer.ready)
            assertTrue(next.localizer.health().startsWith("waiting for start pose"))
        }
        next.hardware.clock.advanceMs(20.0)
        next.robot.initTick()
        assertTrue(next.localizer.ready)
        assertPose(Pose.zero(), next.drive.pose)
        assertNull(next.localizer.fault)
        assertTrue(next.hardware.motors.all { it.writes.isEmpty() })
    }

    @Test
    fun explicitAutonomousStartPoseWinsOverStaleSamples() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        pinpoint.measuredPose = Pose(100.0, 30.0, 1.0)
        pinpoint.staleReadsAfterSetPose = 2
        val start = Pose(8.0, 56.0, Math.PI / 2)

        val auto = Run(pinpoint, startingPose = start)
        val ticks = auto.init(ticks = 4)
        ticks.forEach { assertPose(start, it.first) }
        assertEquals(listOf(false, false, true, true), ticks.map { it.second })
        // Only the start pose is ever written: no zero, no restore.
        assertTrue(pinpoint.setPoseCalls.isNotEmpty())
        assertTrue(pinpoint.setPoseCalls.all { it.x() == start.x() && it.y() == start.y() && it.heading() == start.heading() })
        assertTrue(auto.hardware.motors.all { it.writes.isEmpty() })
    }

    @Test
    fun startPressedBeforeConfirmationKeepsReassertingTheStartPose() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        pinpoint.measuredPose = Pose(100.0, 30.0, 1.0)
        pinpoint.staleReadsAfterSetPose = 1
        val run = Run(pinpoint)
        run.robot.init()
        run.robot.start()
        repeat(3) {
            run.hardware.clock.advanceMs(20.0)
            run.robot.loop()
        }
        assertTrue(run.localizer.ready)
        assertPose(Pose.zero(), run.drive.pose)
    }

    @Test
    fun aPinpointThatNeverAcceptsTheStartPoseFaults() {
        val pinpoint = PedroDriveFixture.OdometryProbe()
        pinpoint.measuredPose = Pose(100.0, 30.0, 1.0)
        pinpoint.staleReadsAfterSetPose = Int.MAX_VALUE
        val run = Run(pinpoint)
        run.robot.init()
        repeat(260) {
            run.hardware.clock.advanceMs(20.0)
            run.robot.initTick()
        }
        assertFalse(run.localizer.ready)
        assertNotNull(run.localizer.fault)
        assertTrue(run.localizer.fault!!.startsWith("start pose not confirmed"))
        assertEquals(0, run.hardware.motors.sumOf { it.writes.size })
    }
}
