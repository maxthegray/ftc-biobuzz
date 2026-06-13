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

    override fun setPose(pose: Pose) {
        poseState = pose
    }

    override fun getPose(): Pose = poseState

    override fun getVelocity(): Vector = velocityState

    override fun breakFollowing() {
        breakFollowingCalls++
    }

    override fun holdPoint(pose: Pose) {
        heldPose = pose
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
