package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import java.lang.reflect.Proxy

/** Real Pedro control and mecanum mixing; only devices and odometry are simulated. */
internal class PedroDriveFixture {
    // SDK tryGet checks the Android device type (and loads native RobotCore).
    // Keep device registration real; replace only that host-incompatible lookup.
    val hardwareMap = object : HardwareMap(null, null) {
        override fun <T> get(type: Class<out T>, name: String): T =
            requireNotNull(tryGet(type, name)) { "Missing device $name" }

        override fun <T> tryGet(type: Class<out T>, name: String): T? =
            allDevicesMap[name]?.firstOrNull { type.isInstance(it) }
                ?.let(type::cast)
    }
    val motors = List(4) { MotorProbe() }
    val localizer = OdometryProbe()
    val follower: Follower
    val drive: MecanumDriveSubsystem

    init {
        listOf("leftFront", "leftRear", "rightFront", "rightRear").forEachIndexed { i, name ->
            hardwareMap.put(name, motors[i].device)
        }
        // Mecanum resolves its sensor through hardwareMap.voltageSensor, which
        // the untyped put() does not populate.
        hardwareMap.voltageSensor.put("voltage", deviceProxy(VoltageSensor::class.java) { name, _ ->
            if (name == "getVoltage") 12.0 else null
        })
        val constants = MecanumConstants().apply {
            xVelocity = 40.0
            yVelocity = 40.0
            frontLeftVector = Vector(1.0, Math.PI / 4.0)
        }
        follower = Follower(FollowerConstants(), localizer, Mecanum(hardwareMap, constants))
        drive = MecanumDriveSubsystem(follower)
    }

    fun clearWrites() = motors.forEach { it.writes.clear() }
    fun powers(): DoubleArray = motors.map { it.power }.toDoubleArray()

    class MotorProbe {
        var power = 0.0
        var direction = DcMotorSimple.Direction.FORWARD
        val writes = mutableListOf<Double>()
        private var motorType = MotorConfigurationType()
        val device = deviceProxy(DcMotorEx::class.java) { name, args ->
            when (name) {
                "getMotorType" -> motorType
                "setMotorType" -> { motorType = args[0] as MotorConfigurationType; null }
                "getPower" -> power
                "setPower" -> {
                    power = args[0] as Double
                    check(power.isFinite() && power in -1.0..1.0) { "Invalid motor power: $power" }
                    writes += power
                    null
                }
                "getDirection" -> direction
                "setDirection" -> { direction = args[0] as DcMotorSimple.Direction; null }
                "getCurrent" -> 0.0
                else -> null
            }
        }
    }

    class OdometryProbe : Localizer {
        var measuredPose = Pose()
        var reads = 0
        var onRead: () -> Unit = {}
        override fun getPose(): Pose = measuredPose
        override fun getVelocity(): Pose = Pose()
        override fun getVelocityVector(): Vector = Vector()
        override fun setStartPose(pose: Pose) { measuredPose = pose }
        override fun setPose(pose: Pose) { measuredPose = pose }
        override fun update() { reads++; onRead() }
        override fun getTotalHeading(): Double = measuredPose.heading
        override fun getForwardMultiplier(): Double = 1.0
        override fun getLateralMultiplier(): Double = 1.0
        override fun getTurningMultiplier(): Double = 1.0
        override fun resetIMU() {}
        override fun getIMUHeading(): Double = measuredPose.heading
        override fun isNAN(): Boolean = !measuredPose.heading.isFinite()
    }
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
