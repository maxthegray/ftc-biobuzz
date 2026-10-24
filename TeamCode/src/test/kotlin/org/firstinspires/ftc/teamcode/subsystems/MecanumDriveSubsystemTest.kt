package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem.TeleopInput
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/** Manual driving through the real Pedro 3 follower and revhub mecanum mixer. */
class MecanumDriveSubsystemTest {

    private class Harness(heading: Double = 0.0) {
        val hardware = PedroDriveFixture()
        val drive = hardware.drive
        val robot = Robot(hardware.hardwareMap, hardware.clock)
        var input = TeleopInput(0.0, 0.0, 0.0)
        val teleop = drive.teleopCommand { input }

        init {
            hardware.localizer.measuredPose = Pose(0.0, 0.0, heading)
            robot.register(drive)
            drive.defaultCommand = teleop
            robot.start()
        }

        fun tick(ms: Double = 20.0) {
            hardware.clock.advanceMs(ms)
            robot.loop()
        }
    }

    /** Wheel powers in mixer order: FL, FR, BL, BR. */
    private fun wheels(forward: Double, strafe: Double, turn: Double): DoubleArray {
        val raw = doubleArrayOf(forward - strafe - turn, forward + strafe + turn, forward + strafe - turn, forward - strafe + turn)
        val max = maxOf(1.0, raw.maxOf { kotlin.math.abs(it) })
        return DoubleArray(4) { raw[it] / max }
    }

    @Test
    fun teleopDrivesTheWheelsOnTheFirstTickWithOneOdometryRead() {
        val h = Harness()
        h.input = TeleopInput(0.5, 0.0, 0.0)
        h.tick()
        // Squared input curve: 0.5 -> 0.25.
        assertArrayEquals(wheels(0.25, 0.0, 0.0), h.hardware.powers(), 1e-9)
        assertEquals(1, h.hardware.localizer.reads)
        h.tick()
        assertEquals(2, h.hardware.localizer.reads)
    }

    @Test
    fun invalidExponentsFallBackToLinearDriveAndStopOnCenteredSticks() {
        val original = DriveConfig.inputExponent
        val h = Harness()
        try {
            for (exponent in doubleArrayOf(0.0, -0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY)) {
                DriveConfig.inputExponent = 2.0
                h.input = TeleopInput(-0.4, 0.3, -0.2)
                h.tick()

                DriveConfig.inputExponent = exponent
                h.input = TeleopInput(0.0, 0.0, 0.0)
                h.tick()
                assertArrayEquals("Centered sticks with exponent $exponent", DoubleArray(4), h.hardware.powers(), 1e-9)

                h.input = TeleopInput(-0.4, 0.3, -0.2)
                h.tick()
                assertArrayEquals("Linear fallback with exponent $exponent", wheels(-0.4, -0.3, 0.2), h.hardware.powers(), 1e-9)
                assertEquals(0, h.robot.commandFaultCount)
            }
        } finally {
            DriveConfig.inputExponent = original
            Scheduler.reset()
        }
    }

    @Test
    fun stickConventionsMapToPedroAxes() {
        val h = Harness()
        h.drive.toggleFieldCentric()
        assertFalse(h.drive.fieldCentric)
        // Stick right = strafe right = Pedro strafe negative.
        h.input = TeleopInput(0.0, 1.0, 0.0)
        h.tick()
        assertArrayEquals(wheels(0.0, -1.0, 0.0), h.hardware.powers(), 1e-9)
        // Stick right turn = clockwise = Pedro turn negative.
        h.input = TeleopInput(0.0, 0.0, 1.0)
        h.tick()
        assertArrayEquals(wheels(0.0, 0.0, -1.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun fieldCentricRotatesStickTranslationByTheMeasuredHeading() {
        val h = Harness(heading = Math.PI / 2)
        assertTrue(h.drive.fieldCentric)
        // Field +x while facing +y is the robot's right.
        h.input = TeleopInput(1.0, 0.0, 0.0)
        h.tick()
        assertArrayEquals(wheels(0.0, -1.0, 0.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun invalidHeadingNeverRotatesTheSticks() {
        val h = Harness(heading = Math.PI / 2)
        h.input = TeleopInput(1.0, 0.0, 0.0)
        h.tick()
        h.hardware.localizer.measuredPose = Pose(0.0, 0.0, Double.NaN)
        h.input = TeleopInput(0.5, 0.0, 0.0)
        h.tick()
        assertArrayEquals(wheels(0.25, 0.0, 0.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun translationMagnitudeIsCappedAtOneLikePedro2() {
        val h = Harness()
        h.drive.toggleFieldCentric()
        h.input = TeleopInput(1.0, -1.0, 0.0)
        h.tick()
        val component = Math.sqrt(0.5)
        assertArrayEquals(wheels(component, component, 0.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun precisionScalesStickInput() {
        val h = Harness()
        h.input = TeleopInput(1.0, 0.0, 0.0, precision = true)
        h.tick()
        assertArrayEquals(wheels(DriveConfig.precisionPowerScale, 0.0, 0.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun anotherDriveCommandTakesOverAndDefaultResumes() {
        val h = Harness()
        h.input = TeleopInput(0.5, 0.0, 0.0)
        h.tick()
        val assist = h.drive.teleopCommand {
            TeleopInput(0.0, 0.0, -1.0) // full CCW turn
        }
        Scheduler.schedule(assist)
        assertFalse(Scheduler.isScheduled(h.teleop))
        h.tick()
        assertArrayEquals(wheels(0.0, 0.0, 1.0), h.hardware.powers(), 1e-9)
        Scheduler.cancel(assist)
        h.tick()
        assertTrue(Scheduler.isScheduled(h.teleop))
        assertArrayEquals(wheels(0.25, 0.0, 0.0), h.hardware.powers(), 1e-9)
    }

    @Test
    fun interruptedTeleopStopsTheFollowerForTheTickWithoutAnOwner() {
        val h = Harness()
        h.input = TeleopInput(1.0, 0.0, 0.0)
        h.tick()
        Scheduler.cancel(h.teleop)
        h.drive.defaultCommand = null
        h.tick()
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        assertEquals("IDLE", h.drive.driveModeName)
    }

    @Test
    fun stopWritesZeroPowerImmediately() {
        val h = Harness()
        h.input = TeleopInput(1.0, 0.0, 0.0)
        h.tick()
        val reads = h.hardware.localizer.reads
        h.robot.stop()
        assertArrayEquals(DoubleArray(4), h.hardware.powers(), 1e-9)
        assertEquals(reads, h.hardware.localizer.reads)
    }

    @Test
    fun fieldCentricRuntimeStateDoesNotMutateThePersistedDefault() {
        val original = DriveConfig.fieldCentricDefault
        try {
            DriveConfig.fieldCentricDefault = false
            val drive = PedroDriveFixture().drive
            assertFalse(drive.fieldCentric)
            drive.toggleFieldCentric()
            assertTrue(drive.fieldCentric)
            assertFalse(DriveConfig.fieldCentricDefault)
        } finally {
            DriveConfig.fieldCentricDefault = original
        }
    }

    @Test
    fun motorChannelsLogCommandedPowerAndEncoderVelocityEveryTick() {
        val h = Harness()
        h.hardware.motors.forEachIndexed { i, motor -> motor.velocity = 100.0 * (i + 1) }
        h.input = TeleopInput(0.5, 0.0, 0.3)
        h.tick()
        val log = RecordingStateLog()
        h.drive.logState(log)
        assertTrue(log.channels.containsKey("fieldCentric"))
        assertTrue(log.channels.containsKey("odometryFallback"))
        assertArrayEquals(h.hardware.powers(), log.channels["motors/power"] as DoubleArray, 1e-9)
        assertArrayEquals(doubleArrayOf(100.0, 200.0, 300.0, 400.0), log.channels["motors/velocityTps"] as DoubleArray, 1e-9)
    }

    @Test
    fun loggedPowerIsZeroOnceTheDrivetrainStops() {
        val h = Harness()
        h.input = TeleopInput(1.0, 0.0, 0.0)
        h.tick()
        Scheduler.cancel(h.teleop)
        h.drive.defaultCommand = null
        h.tick()
        val log = RecordingStateLog()
        h.drive.logState(log)
        assertArrayEquals(DoubleArray(4), log.channels["motors/power"] as DoubleArray, 1e-9)

        val stopped = Harness()
        stopped.input = TeleopInput(1.0, 0.0, 0.0)
        stopped.tick()
        stopped.robot.stop()
        stopped.drive.logState(log)
        assertArrayEquals(DoubleArray(4), log.channels["motors/power"] as DoubleArray, 1e-9)
    }

    @Test
    fun currentIsSampledOneMotorAtATimeRoundRobin() {
        val h = Harness()
        val log = RecordingStateLog()
        h.tick()
        h.drive.logState(log)
        assertTrue(log.channels.containsKey("motors/leftFront/currentAmps"))
        log.channels.clear()
        h.tick(100.0)
        h.drive.logState(log)
        assertFalse(log.channels.keys.any { it.endsWith("currentAmps") })
        h.tick(100.0)
        h.drive.logState(log)
        assertTrue(log.channels.containsKey("motors/rightFront/currentAmps"))
    }

    @Test
    fun aCurrentSampleWaitsForTheRecorderAndIsLoggedOnce() {
        val h = Harness()
        val log = RecordingStateLog()
        h.tick()
        h.tick(10.0)
        h.drive.logState(log)
        assertTrue(log.channels.containsKey("motors/leftFront/currentAmps"))
        log.channels.clear()
        h.drive.logState(log)
        assertFalse(log.channels.keys.any { it.endsWith("currentAmps") })
    }

    private class RecordingStateLog : StateLog {
        val channels = mutableMapOf<String, Any>()
        override fun put(channel: String, value: Double) { channels[channel] = value }
        override fun put(channel: String, value: Long) { channels[channel] = value }
        override fun put(channel: String, value: Boolean) { channels[channel] = value }
        override fun put(channel: String, value: String) { channels[channel] = value }
        override fun put(channel: String, value: DoubleArray) { channels[channel] = value }
    }
}
