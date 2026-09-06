package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.junit.Assert.*
import org.junit.Test

class PedroTeleopStartupTest {
    @Test
    fun firstStickSampleDrivesWheelsWithOneOdometryRead() {
        val h = PedroDriveFixture()
        h.clearWrites()
        val teleop = h.drive.teleopCommand { TeleopInput(0.5, 0.0, 0.0) }
        teleop.start()
        teleop.execute()
        assertEquals(0, h.localizer.reads)
        assertTrue(h.motors.all { it.writes.isEmpty() })

        h.drive.writeHardware()
        assertEquals(1, h.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.powers(), 1e-9)

        teleop.execute()
        h.drive.writeHardware()
        assertEquals(2, h.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.powers(), 1e-9)
    }

    @Test
    fun pathInterruptionReturnsSticksOnTheFirstResumeTick() {
        val h = PedroDriveFixture()
        val robot = Robot(h.hardwareMap)
        robot.register(h.drive)
        h.drive.defaultCommand = h.drive.teleopCommand { TeleopInput(0.5, 0.0, 0.0) }
        robot.start()
        robot.loop()
        val hold = h.drive.holdCommand(Pose2d(10.0, 0.0, 0.0))
        robot.scheduler.schedule(hold)
        robot.loop()
        robot.scheduler.cancel(hold)
        val reads = h.localizer.reads

        robot.loop()

        assertEquals(reads + 1, h.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.powers(), 1e-9)
        assertEquals(0, robot.commandFaultCount)
    }

    @Test
    fun startupUsesTheMeasuredHeadingForFieldCentricDrive() {
        val h = PedroDriveFixture()
        h.localizer.measuredPose = Pose(0.0, 0.0, Math.PI / 2.0)
        val teleop = h.drive.teleopCommand { TeleopInput(1.0, 0.0, 0.0) }
        teleop.start()
        teleop.execute()
        h.drive.writeHardware()

        assertEquals(1, h.localizer.reads)
        assertArrayEquals(doubleArrayOf(-1.0, 1.0, 1.0, -1.0), h.powers(), 1e-9)
    }

    @Test
    fun cancelledStartupCannotOverrideAPathScheduledInTheSameTick() {
        val h = PedroDriveFixture()
        val teleop = h.drive.teleopCommand { TeleopInput(1.0, 0.0, 0.0) }
        teleop.start()
        teleop.execute()
        teleop.end(EndCondition.INTERRUPTED)
        h.drive.holdCommand(Pose2d.ZERO).start()
        h.drive.writeHardware()

        assertFalse(h.follower.teleopDrive)
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, h.drive.mode)
        assertArrayEquals(DoubleArray(4), h.powers(), 1e-9)
        assertEquals(1, h.localizer.reads)
    }

    @Test
    fun stoppingBeforeTheFirstUpdateZerosAllMotors() {
        val h = PedroDriveFixture()
        h.clearWrites()
        h.drive.stop()
        assertTrue(h.motors.all { it.writes == listOf(0.0) })
        assertEquals(0, h.localizer.reads)
    }
}
