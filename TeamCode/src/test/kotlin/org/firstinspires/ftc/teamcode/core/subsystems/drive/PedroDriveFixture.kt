package org.firstinspires.ftc.teamcode.core.subsystems.drive

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
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.lang.reflect.Proxy
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Real Pedro 3 [Follower], Foresight and revhub [Mecanum] mixing on the host.
 * Only the motors and odometry are simulated: motor probes record every power
 * write, and [OdometryProbe] reports whatever pose the test places.
 */
internal class PedroDriveFixture(tuned: Boolean = true) {
    // SDK tryGet checks the Android device type (and loads native RobotCore).
    // Keep device registration real; replace only that host-incompatible lookup.
    val hardwareMap = object : HardwareMap(null, null) {
        override fun <T> get(type: Class<out T>, name: String): T =
            requireNotNull(tryGet(type, name)) { "Missing device $name" }

        override fun <T> tryGet(type: Class<out T>, name: String): T? =
            allDevicesMap[name]?.firstOrNull { type.isInstance(it) }?.let(type::cast)
    }

    /** In mixer order: front left, front right, back left, back right. */
    val motors = List(4) { MotorProbe() }
    val localizer = OdometryProbe()
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

    class MotorProbe {
        var power = 0.0
        var direction = DcMotorSimple.Direction.FORWARD
        var zeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN
        val writes = mutableListOf<Double>()
        val device = deviceProxy(DcMotorEx::class.java) { name, args ->
            when (name) {
                "getPower" -> power
                "setPower" -> {
                    power = args[0] as Double
                    check(power.isFinite() && power in -1.0..1.0) { "Invalid motor power: $power" }
                    writes += power
                    null
                }
                "getDirection" -> direction
                "setDirection" -> { direction = args[0] as DcMotorSimple.Direction; null }
                "setZeroPowerBehavior" -> { zeroPowerBehavior = args[0] as DcMotor.ZeroPowerBehavior; null }
                "getCurrent" -> 0.0
                else -> null
            }
        }
    }

    class OdometryProbe : Localizer {
        var measuredPose: Pose = Pose.zero()
        var measuredVelocity: Velocity = Velocity.zero()
        var reads = 0
        var onRead: () -> Unit = {}
        override fun setPose(pose: Pose) { measuredPose = pose }
        override fun state(): MotionState = MotionState.ofVelocity(measuredPose, measuredVelocity)
        override fun update() { reads++; onRead() }
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

internal fun <T : Any> deviceProxy(type: Class<T>, call: (String, Array<out Any?>) -> Any?): T =
    requireNotNull(type.cast(Proxy.newProxyInstance(type.classLoader, arrayOf(type)) { proxy, method, args ->
        when (method.name) {
            "hashCode" -> System.identityHashCode(proxy)
            "equals" -> proxy === args?.get(0)
            "toString", "getDeviceName", "getConnectionInfo" -> type.simpleName
            else -> call(method.name, args ?: emptyArray()) ?: when (method.returnType) {
                java.lang.Boolean.TYPE -> false
                java.lang.Integer.TYPE -> 0
                java.lang.Double.TYPE -> 0.0
                else -> null
            }
        }
    }))
