package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** The TeleOpBase localizer-fault policy: robot-centric sticks with no odometry. */
class TeleopWatchdogRecoveryTest {
    private class Harness {
        val hardware = PedroDriveFixture()
        val drive = hardware.drive
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        var input = TeleopInput(0.5, 0.0, 0.0)
        val normal = drive.teleopCommand { input }
        val fallback = drive.robotCentricFallbackCommand { input }
        val localizer: LocalizerSubsystem

        init {
            robot.register(drive)
            drive.defaultCommand = normal
            localizer = robot.register(
                LocalizerSubsystem(
                    hardware.follower,
                    hardware.clock,
                    isFollowing = drive::isFollowing,
                    onFault = {
                        drive.defaultCommand = fallback
                        Scheduler.schedule(fallback)
                    },
                ),
            )
            robot.start()
        }

        fun tick() {
            hardware.clock.advanceMs(20.0)
            robot.loop()
        }

        fun failOdometry() {
            hardware.localizer.measuredPose = Pose(0.0, 0.0, Double.NaN)
            // After detection, any attempted sensor read is a failure.
            hardware.localizer.onRead = { error("Pinpoint disconnected") }
        }
    }

    @Test
    fun nanHeadingHandsDrivingToSensorIndependentRobotCentricWheelControl() {
        val h = Harness()
        h.tick()
        val reads = h.hardware.localizer.reads
        h.failOdometry()
        h.tick()

        assertNotNull(h.localizer.fault)
        assertFalse(Scheduler.isScheduled(h.normal))
        assertTrue(Scheduler.isScheduled(h.fallback))
        assertTrue(h.drive.odometryFallback)
        assertEquals("ROBOT_CENTRIC_FALLBACK", h.drive.driveModeName)
        assertEquals(reads, h.hardware.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.25 }, h.hardware.powers(), 1e-9)

        // Field-centric is on, but the fallback never rotates by a heading.
        assertTrue(h.drive.fieldCentric)
        h.input = TeleopInput(0.0, 1.0, 0.0)
        h.tick()
        assertArrayEquals(doubleArrayOf(1.0, -1.0, -1.0, 1.0), h.hardware.powers(), 1e-9)
        h.input = TeleopInput(0.0, 0.0, 1.0)
        h.tick()
        assertArrayEquals(doubleArrayOf(1.0, -1.0, 1.0, -1.0), h.hardware.powers(), 1e-9)
        h.input = TeleopInput(0.0, 0.0, 0.0)
        h.tick()
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        assertEquals(reads, h.hardware.localizer.reads)
        assertEquals(0, h.robot.commandFaultCount)
    }

    @Test
    fun faultPreemptsAGroupedAssistAndAPathAndBlocksReentry() {
        val h = Harness()
        var ended: EndCondition? = null
        val assist = h.drive.teleopCommand(priority = CommandPriorities.AUTON_ROUTINE, onEnd = { ended = it }) {
            TeleopInput(0.0, 0.0, 0.0, turnPower = 0.2)
        }
        val group = sequential(assist)
        Scheduler.schedule(group)
        h.tick()

        h.failOdometry()
        h.tick()

        assertEquals(EndCondition.INTERRUPTED, ended)
        assertFalse(Scheduler.isScheduled(group))
        Scheduler.schedule(assist)
        assertFalse(Scheduler.isScheduled(assist))
        val follow = h.drive.followCommand(Paths.line(Pose(0.0, 0.0), Pose(24.0, 0.0)).constant(0.0))
        Scheduler.schedule(follow)
        assertFalse(Scheduler.isScheduled(follow))
        assertTrue(Scheduler.isScheduled(h.fallback))
        assertArrayEquals(DoubleArray(4) { 0.25 }, h.hardware.powers(), 1e-9)
    }

    @Test
    fun fallbackCanStartBeforePedroHasEverUpdatedAndStopWithoutReadingOdometry() {
        val h = Harness()
        h.failOdometry()
        h.tick()
        assertEquals(0, h.hardware.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.25 }, h.hardware.powers(), 1e-9)
        assertEquals(
            listOf(Direction.REVERSE, Direction.FORWARD, Direction.REVERSE, Direction.FORWARD),
            h.hardware.motors.map { it.direction },
        )

        h.robot.stop()
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        assertEquals(0, h.hardware.localizer.reads)
    }

    @Test
    fun fallbackPreservesPrecisionAndBoundsCombinedInputs() {
        val h = Harness()
        h.input = TeleopInput(1.0, 0.0, 0.0, precision = true)
        h.tick()
        val expected = h.hardware.powers()
        h.failOdometry()
        h.tick()
        assertArrayEquals(expected, h.hardware.powers(), 1e-9)
        h.input = TeleopInput(1.0, 1.0, 1.0)
        h.tick()
        assertTrue(h.hardware.powers().all { it.isFinite() && it in -1.0..1.0 })
    }
}
