package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.command.Groups
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.junit.Assert.*
import org.junit.Test

class TeleopWatchdogRecoveryTest {
    private class Harness {
        val hardware = PedroDriveFixture()
        val clock = FakeClock()
        val drive = hardware.drive
        val robot = Robot(hardware.hardwareMap, clock)
        var input = TeleopInput(0.5, 0.0, 0.0)
        val normal = drive.teleopCommand { input }
        val fallback = drive.robotCentricFallbackCommand { input }
        val localizer: LocalizerSubsystem

        init {
            robot.containCommandFaults = true
            robot.register(drive)
            drive.defaultCommand = normal
            localizer = robot.register(LocalizerSubsystem(
                hardware.follower, clock,
                isFollowing = drive::isFollowing,
                onFault = {
                    drive.defaultCommand = fallback
                    robot.scheduler.schedule(fallback)
                },
            ))
            robot.start()
        }

        fun tick() { clock.advanceMs(20.0); robot.loop() }

        fun failOdometry() {
            // Poison Pedro's reported heading without refreshing its pose cache.
            hardware.follower.poseTracker.setHeadingOffset(Double.NaN)
            // After detection, even an attempted sensor read is a failure.
            hardware.localizer.onRead = { error("Pinpoint disconnected") }
        }
    }

    @Test
    fun nanHeadingHandsDefaultDrivingToFiniteSensorIndependentWheelControl() {
        val h = Harness()
        h.tick()
        val reads = h.hardware.localizer.reads
        h.failOdometry()
        h.tick()

        assertNotNull(h.localizer.fault)
        assertFalse(h.robot.scheduler.isScheduled(h.normal))
        assertTrue(h.robot.scheduler.isScheduled(h.fallback))
        assertTrue(h.drive.odometryFallback)
        assertEquals(reads, h.hardware.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.hardware.powers(), 1e-9)

        h.drive.toggleFieldCentric()
        h.input = TeleopInput(0.0, 1.0, 0.0)
        h.tick()
        assertArrayEquals(doubleArrayOf(-1.0, 1.0, 1.0, -1.0), h.hardware.powers(), 1e-9)
        h.input = TeleopInput(0.0, 0.0, 1.0)
        h.tick()
        assertArrayEquals(doubleArrayOf(1.0, 1.0, -1.0, -1.0), h.hardware.powers(), 1e-9)
        h.input = TeleopInput(0.0, 0.0, 0.0)
        h.tick()
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        assertEquals(reads, h.hardware.localizer.reads)
        assertEquals(0, h.robot.commandFaultCount)
    }

    @Test
    fun faultPreemptsGroupedAssistAndBlocksReentryWhileKeepingSticks() {
        val h = Harness()
        var ended: EndCondition? = null
        val assist = h.drive.teleopCommand(
            priority = CommandPriorities.AUTON_ROUTINE,
            onEnd = { ended = it },
        ) { TeleopInput(0.0, 0.0, 0.0, turnPower = 0.2) }
        val group = Groups.sequential(assist)
        h.robot.scheduler.schedule(group)
        h.tick()

        h.failOdometry()
        h.tick()

        assertEquals(EndCondition.INTERRUPTED, ended)
        assertFalse(h.robot.scheduler.isScheduled(group))
        assertFalse(h.robot.scheduler.schedule(assist))
        assertFalse(h.robot.scheduler.schedule(h.drive.holdCommand(Pose2d.ZERO)))
        assertTrue(h.robot.scheduler.isScheduled(h.fallback))
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.hardware.powers(), 1e-9)
    }

    @Test
    fun fallbackCanStartBeforePedroHasEverUpdatedAndStopWithoutReadingOdometry() {
        val h = Harness()
        h.failOdometry()
        h.tick()
        assertEquals(0, h.hardware.localizer.reads)
        assertArrayEquals(DoubleArray(4) { 0.3535533905932738 }, h.hardware.powers(), 1e-9)
        assertEquals(
            listOf(Direction.REVERSE, Direction.REVERSE, Direction.FORWARD, Direction.FORWARD),
            h.hardware.motors.map { it.direction },
        )

        h.drive.stop()
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
