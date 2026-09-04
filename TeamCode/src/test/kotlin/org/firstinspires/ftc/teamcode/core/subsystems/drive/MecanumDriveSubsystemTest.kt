package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class MecanumDriveSubsystemTest {

    @Test
    fun trackDriveModeAddsDriveRequirementAndPreservesWrappedRequirements() {
        val drive = MecanumDriveSubsystem(fakeFollower())
        val extra = Any()
        val wrapped = CommandBuilder()
            .requiring(extra)
            .setPriority(CommandPriorities.DRIVER_ACTION)

        val command = drive.trackDriveMode(
            wrapped,
            running = MecanumDriveSubsystem.Mode.FOLLOWING,
            finished = MecanumDriveSubsystem.Mode.IDLE,
        )

        assertTrue(command.requirements().contains(drive))
        assertTrue(command.requirements().contains(extra))
        assertEquals(CommandPriorities.DRIVER_ACTION, command.priority())
    }

    @Test
    fun interruptedTrackedCommandBreaksFollowingAndIdles() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val command = drive.trackDriveMode(
            CommandBuilder().setDone { false },
            running = MecanumDriveSubsystem.Mode.FOLLOWING,
            finished = MecanumDriveSubsystem.Mode.HOLDING,
        )

        command.start()
        assertEquals(MecanumDriveSubsystem.Mode.FOLLOWING, drive.mode)
        val callsBeforeEnd = follower.breakFollowingCalls

        command.end(EndCondition.INTERRUPTED)
        assertEquals(callsBeforeEnd + 1, follower.breakFollowingCalls)
        assertEquals(MecanumDriveSubsystem.Mode.IDLE, drive.mode)
    }

    @Test
    fun naturallyEndedTrackedCommandKeepsFinishedMode() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val command = drive.trackDriveMode(
            CommandBuilder().setDone { true },
            running = MecanumDriveSubsystem.Mode.FOLLOWING,
            finished = MecanumDriveSubsystem.Mode.HOLDING,
        )

        command.start()
        val callsBeforeEnd = follower.breakFollowingCalls
        command.end(EndCondition.NATURALLY)

        assertEquals(callsBeforeEnd, follower.breakFollowingCalls)
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, drive.mode)
    }

    @Test
    fun holdCommandHoldsTheRequestedPoseIncludingHeading() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val target = Pose2d(10.0, 20.0, 1.5)

        drive.holdCommand(target).start()

        val held = follower.heldPose
        assertTrue(held != null)
        assertEquals(target.x, held!!.x, 1e-9)
        assertEquals(target.y, held.y, 1e-9)
        assertEquals(target.heading, held.heading, 1e-9)
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, drive.mode)
    }

    @Test
    fun teleopEnableRunsExactlyOneFollowerUpdateThatTick() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val teleop = drive.teleopCommand { MecanumDriveSubsystem.TeleopInput(0.0, 0.0, 0.0) }

        teleop.start()
        teleop.execute()
        drive.writeHardware()

        // startTeleOpDrive() updates internally; writeHardware() must not
        // update again on the enable tick.
        assertEquals(1, follower.startTeleopDriveCalls)
        assertEquals(1, follower.updateCalls)

        teleop.execute()
        drive.writeHardware()
        assertEquals(1, follower.startTeleopDriveCalls)
        assertEquals(2, follower.updateCalls)
    }

    @Test
    fun turnPowerBypassesTheStickCurveAndTheStickNegation() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        // A small assist output is exactly what the squared stick curve would
        // destroy (0.1^2 = 0.01), and it is already CCW-positive, so it must
        // reach the follower untouched by the sign flip stick turn gets.
        val teleop = drive.teleopCommand {
            MecanumDriveSubsystem.TeleopInput(0.0, 0.0, turn = 0.0, turnPower = 0.1)
        }

        teleop.start()
        teleop.execute()

        assertEquals(0.1, follower.lastTeleOpDrive!![2], 1e-9)
    }

    @Test
    fun turnPowerTakesPrecedenceOverStickTurn() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val teleop = drive.teleopCommand {
            MecanumDriveSubsystem.TeleopInput(0.0, 0.0, turn = 1.0, turnPower = -0.2)
        }

        teleop.start()
        teleop.execute()

        assertEquals(-0.2, follower.lastTeleOpDrive!![2], 1e-9)
    }

    @Test
    fun nonFiniteTurnPowerCommandsZeroInsteadOfNaNingTheMotors() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val teleop = drive.teleopCommand {
            MecanumDriveSubsystem.TeleopInput(0.0, 0.0, 0.0, turnPower = Double.NaN)
        }

        teleop.start()
        teleop.execute()

        assertEquals(0.0, follower.lastTeleOpDrive!![2], 1e-9)
    }

    @Test
    fun teleopCommandStartHooksComposeWithTheTeleopEnable() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        var starts = 0
        var endCondition: EndCondition? = null
        val teleop = drive.teleopCommand(
            name = "assist",
            priority = CommandPriorities.AUTON_ROUTINE,
            onStart = { starts++ },
            onEnd = { endCondition = it },
        ) { MecanumDriveSubsystem.TeleopInput(0.0, 0.0, 0.0) }

        teleop.start()
        drive.writeHardware()
        teleop.end(EndCondition.INTERRUPTED)

        assertEquals(1, starts)
        // The hook must not have replaced enableTeleop(), or an assist
        // preempting a path would never enter manual drive mode.
        assertEquals(1, follower.startTeleopDriveCalls)
        assertEquals(CommandPriorities.AUTON_ROUTINE, teleop.priority())
        assertEquals("assist", teleop.toString())
        assertEquals(EndCondition.INTERRUPTED, endCondition)
    }

    @Test
    fun holdCommandWaitsForAFollowerUpdateBeforeTrustingErrors() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val hold = drive.holdCommand(Pose2d(24.0, 0.0, 0.0))

        hold.start()
        // No update has run since holdPoint(): Pedro's cached errors are
        // stale (or null on a first-ever hold) — done() must neither read
        // them (the fake throws) nor complete.
        assertFalse(hold.done())

        follower.translationalErrorMagnitude = 24.0
        drive.writeHardware()
        assertFalse(hold.done())

        follower.translationalErrorMagnitude = 0.0
        follower.headingErrorValue = 0.0
        drive.writeHardware()
        assertTrue(hold.done())
    }

    @Test
    fun holdCommandTimesOutWhenThePoseRemainsUnreachable() {
        val clock = FakeClock()
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower, clock)
        val hold = drive.holdCommand(Pose2d(24.0, 0.0, 0.0), timeoutMs = 500.0)

        hold.start()
        follower.translationalErrorMagnitude = 24.0
        drive.writeHardware()
        assertFalse(hold.done())

        clock.advanceMs(600.0)
        assertTrue(hold.done())
    }

    @Test
    fun pathProgressLatchesActualValuesWithoutSynthesizingCompletion() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val command = drive.trackDriveMode(
            CommandBuilder().setDone { !follower.busyValue },
            running = MecanumDriveSubsystem.Mode.FOLLOWING,
            finished = MecanumDriveSubsystem.Mode.HOLDING,
        )
        follower.busyValue = true
        command.start()

        follower.pathProgressValue = 0.65
        assertEquals(0.65, drive.pathProgress(), 1e-9)
        follower.pathProgressValue = 0.4
        assertEquals(0.65, drive.pathProgress(), 1e-9)

        follower.busyValue = false
        drive.periodic()
        assertEquals(MecanumDriveSubsystem.Mode.HOLDING, drive.mode)
        assertEquals(0.65, drive.pathProgress(), 1e-9)
    }

    @Test
    fun pathProgressReturnsZeroAfterCancellation() {
        val follower = fakeFollower()
        val drive = MecanumDriveSubsystem(follower)
        val command = drive.trackDriveMode(
            CommandBuilder().setDone { false },
            running = MecanumDriveSubsystem.Mode.FOLLOWING,
            finished = MecanumDriveSubsystem.Mode.HOLDING,
        )
        follower.busyValue = true
        command.start()
        follower.pathProgressValue = 0.5
        assertEquals(0.5, drive.pathProgress(), 1e-9)

        command.end(EndCondition.INTERRUPTED)

        assertEquals(0.0, drive.pathProgress(), 0.0)
    }

    @Test
    fun fieldCentricRuntimeStateDoesNotMutateThePersistedDefault() {
        val original = DriveConfig.fieldCentricDefault
        try {
            DriveConfig.fieldCentricDefault = false
            val drive = MecanumDriveSubsystem(fakeFollower())

            assertFalse(drive.fieldCentric)
            drive.toggleFieldCentric()
            assertTrue(drive.fieldCentric)
            assertFalse(DriveConfig.fieldCentricDefault)
        } finally {
            DriveConfig.fieldCentricDefault = original
        }
    }

    @Test
    fun logStateWithoutRealMotorsWritesNoMotorChannels() {
        // The fake follower's drivetrain is not Pedro's Mecanum, so init
        // resolves no motors — motor telemetry must degrade to a no-op. The
        // field-centric drive-mode flag is config, not hardware, so it logs
        // regardless.
        val drive = MecanumDriveSubsystem(fakeFollower())
        drive.init(com.qualcomm.robotcore.hardware.HardwareMap(null, null))
        val log = RecordingStateLog()

        drive.periodic()
        drive.logState(log)

        assertTrue(log.channels.containsKey("fieldCentric"))
        assertFalse(log.channels.keys.any { it.startsWith("motors/") })
    }

    private class RecordingStateLog : org.firstinspires.ftc.teamcode.core.logging.StateLog {
        val channels = mutableMapOf<String, Any>()
        override fun put(channel: String, value: Double) { channels[channel] = value }
        override fun put(channel: String, value: Long) { channels[channel] = value }
        override fun put(channel: String, value: Boolean) { channels[channel] = value }
        override fun put(channel: String, value: String) { channels[channel] = value }
    }
}

internal fun fakeFollower(): FakeFollower = FakeFollower()

internal class FakeFollower : Follower(FollowerConstants(), FakeLocalizer(), FakeDrivetrain()) {
    private var poseState = Pose()
    private val velocityState = Vector()

    var breakFollowingCalls = 0
        private set
    var heldPose: Pose? = null
        private set
    var updateCalls = 0
        private set
    var startTeleopDriveCalls = 0
        private set

    /** Error values Pedro would cache; recomputed only by [update] on the real thing. */
    var translationalErrorMagnitude = 0.0
    var headingErrorValue = 0.0
    var busyValue = false
    var pathProgressValue = 0.0

    override fun setPose(pose: Pose) {
        poseState = pose
    }

    override fun getPose(): Pose = poseState

    override fun getVelocity(): Vector = velocityState

    override fun breakFollowing() {
        breakFollowingCalls++
        busyValue = false
    }

    override fun isBusy(): Boolean = busyValue

    override fun getCurrentPathNumber(): Double = 0.0

    override fun getCurrentTValue(): Double = pathProgressValue

    override fun holdPoint(pose: Pose) {
        heldPose = pose
    }

    // The base implementation stores into VectorCalculator state that only
    // exists after a real breakFollowing() has run — which this fake
    // deliberately intercepts. Recording the values is all tests need.
    var lastTeleOpDrive: DoubleArray? = null
        private set

    override fun setTeleOpDrive(forward: Double, strafe: Double, turn: Double, isRobotCentric: Boolean) {
        lastTeleOpDrive = doubleArrayOf(forward, strafe, turn)
    }

    override fun update() {
        updateCalls++
    }

    // Pedro 2.1.1 runs a full update() inside startTeleopDrive — emulate it
    // so a double-update on the teleop enable tick is visible to tests.
    override fun startTeleopDrive(useBrakeMode: Boolean) {
        startTeleopDriveCalls++
        update()
    }

    override fun startTeleOpDrive(useBrakeMode: Boolean) = startTeleopDrive(useBrakeMode)

    // Pedro computes these from fields that stay null until the first-ever
    // update() — reading earlier NPEs. Emulate that hazard.
    override fun getTranslationalError(): Vector {
        check(updateCalls > 0) { "Pedro NPEs when errors are read before the first update()" }
        return Vector(translationalErrorMagnitude, 0.0)
    }

    override fun getHeadingError(): Double {
        check(updateCalls > 0) { "Pedro NPEs when errors are read before the first update()" }
        return headingErrorValue
    }
}

private class FakeLocalizer : Localizer {
    private var pose = Pose()

    override fun getPose(): Pose = pose
    override fun getVelocity(): Pose = Pose()
    override fun getVelocityVector(): Vector = Vector()
    override fun setStartPose(setStart: Pose) { pose = setStart }
    override fun setPose(setPose: Pose) { pose = setPose }
    override fun update() {}
    override fun getTotalHeading(): Double = pose.heading
    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0
    override fun resetIMU() {}
    override fun getIMUHeading(): Double = pose.heading
    override fun isNAN(): Boolean = false
}

private class FakeDrivetrain : Drivetrain() {
    override fun calculateDrive(
        correctivePower: Vector,
        headingPower: Vector,
        drivePower: Vector,
        robotHeading: Double,
    ): DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    override fun updateConstants() {}
    override fun breakFollowing() {}
    override fun runDrive(powers: DoubleArray) {}
    override fun startTeleopDrive() {}
    override fun startTeleopDrive(brake: Boolean) {}
    override fun xVelocity(): Double = 0.0
    override fun yVelocity(): Double = 0.0
    override fun setXVelocity(xVelocity: Double) {}
    override fun setYVelocity(yVelocity: Double) {}
    override fun getVoltage(): Double = 12.0
    override fun debugString(): String = "fake"
}
