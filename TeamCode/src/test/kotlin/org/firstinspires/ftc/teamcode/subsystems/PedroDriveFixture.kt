package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.algorithm.Foresight
import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.controllers.Controller
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Localizer
import com.pedropathing.localization.MotionState
import com.pedropathing.math.Matrix
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.math.Velocity
import com.pedropathing.revhub.drivetrains.Mecanum
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Real Pedro 3 [Follower], Foresight and revhub [Mecanum] mixing on the host.
 * Only the motors and odometry are simulated: motor probes record every power
 * write, and [OdometryProbe] reports whatever pose the test places.
 */
internal class PedroDriveFixture(
    tuned: Boolean = true,
    /** Pass the previous run's probe to model the same Pinpoint across op-modes. */
    val localizer: OdometryProbe = OdometryProbe(),
) {
    val hardwareMap = FakeHardwareMap()

    /** In mixer order: front left, front right, back left, back right. */
    val motors = List(4) { MotorProbe() }
    val clock = FakeClock()
    val follower: Follower
    val drive: MecanumDriveSubsystem

    init {
        listOf(
            RobotConfig.Drive.FRONT_LEFT_MOTOR,
            RobotConfig.Drive.FRONT_RIGHT_MOTOR,
            RobotConfig.Drive.BACK_LEFT_MOTOR,
            RobotConfig.Drive.BACK_RIGHT_MOTOR,
        ).forEachIndexed { i, name -> hardwareMap.put(name, motors[i].device) }
        follower = Follower(
            localizer,
            Mecanum(hardwareMap, Constants.drivetrainConfig),
            if (tuned) Foresight(testForesightConfig()) else null,
        )
        drive = MecanumDriveSubsystem(follower, clock)
        drive.init(hardwareMap)
    }

    fun clearWrites() = motors.forEach { it.writes.clear() }
    fun powers(): DoubleArray = motors.map { it.power }.toDoubleArray()

    /**
     * A Pinpoint as Pedro's PinpointLocalizer sees it: the device keeps its
     * pose between op-modes, [setPose] writes the device and the cached state,
     * and [update] replaces the cache with what the device reports. With
     * [staleReadsAfterSetPose] > 0 the device reports its old pose for that many
     * reads after a write, like a sample taken before the write landed;
     * [Int.MAX_VALUE] models a device that never accepts the write.
     */
    class OdometryProbe : Localizer {
        /** What the hardware holds. */
        var devicePose: Pose = Pose.zero()
        private var cachedPose: Pose = Pose.zero()

        /** The robot is physically here: sets the device and the cached state. */
        var measuredPose: Pose
            get() = cachedPose
            set(value) {
                devicePose = value
                cachedPose = value
            }
        var measuredVelocity: Velocity = Velocity.zero()
        var reads = 0
        var onRead: () -> Unit = {}
        var staleReadsAfterSetPose = 0
        val setPoseCalls = mutableListOf<Pose>()
        private var pendingWrite: Pose? = null
        private var pendingStaleReads = 0

        override fun setPose(pose: Pose) {
            setPoseCalls += pose
            cachedPose = pose
            if (staleReadsAfterSetPose == 0) {
                devicePose = pose
            } else {
                // A repeated write while one is in flight replaces it but does not restart the delay.
                if (pendingWrite == null) pendingStaleReads = staleReadsAfterSetPose
                pendingWrite = pose
            }
        }

        override fun state(): MotionState = MotionState.ofVelocity(cachedPose, measuredVelocity)

        override fun update() {
            reads++
            onRead()
            pendingWrite?.let {
                if (pendingStaleReads > 0) {
                    pendingStaleReads--
                } else {
                    devicePose = it
                    pendingWrite = null
                }
            }
            cachedPose = devicePose
        }

        override fun reset() {}
    }
}

/** Arbitrary Foresight gains for host tests only. Never used on a robot. */
internal fun testForesightConfig(): ForesightConfig = ForesightConfig { c ->
    c.forwardTranslational.set(Controller.proportional(0.1))
    c.strafeTranslational.set(Controller.proportional(0.1))
    c.coast.set(Controller.proportionalFeedforward(0.01))
    c.brake.set(Controller.proportionalFeedforward(0.01))
    c.headingFeedback.set(Controller.proportional(1.0))
    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.005))
    c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1))
    c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001))
    c.maxAchievableForwardVelocity.set(60.0)
    c.maxAchievableStrafeVelocity.set(50.0)
    c.naturalForwardDeceleration.set(80.0)
    c.naturalStrafeDeceleration.set(90.0)
}
