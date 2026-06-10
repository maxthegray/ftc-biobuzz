package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.junit.Assert.assertEquals
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
}

internal fun fakeFollower(): Follower = FakeFollower()

private class FakeFollower : Follower(FollowerConstants(), FakeLocalizer(), FakeDrivetrain()) {
    private var poseState = Pose()
    private val velocityState = Vector()

    override fun setPose(pose: Pose) {
        poseState = pose
    }

    override fun getPose(): Pose = poseState

    override fun getVelocity(): Vector = velocityState
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
