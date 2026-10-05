package org.firstinspires.ftc.teamcode.core.sim

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.lang.reflect.Proxy

/**
 * A real SDK [HardwareMap] for host tests: register fakes with `put(name, probe.device)`.
 * Only the device lookup is replaced, because the SDK's checks the Android
 * device type (and loads native RobotCore).
 */
class FakeHardwareMap : HardwareMap(null, null) {
    override fun <T> get(type: Class<out T>, name: String): T =
        requireNotNull(tryGet(type, name)) { "Missing device $name" }

    override fun <T> tryGet(type: Class<out T>, name: String): T? =
        allDevicesMap[name]?.firstOrNull { type.isInstance(it) }?.let(type::cast)
}

/** A [DcMotorEx] that records every power written. */
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

/** A [Servo] that records its position; [beforeWrite] can model a slow write. */
class ServoProbe(private val beforeWrite: (from: Double, to: Double) -> Unit = { _, _ -> }) {
    var position = Double.NaN
    val device = deviceProxy(Servo::class.java) { name, args ->
        when (name) {
            "getPosition" -> position
            "setPosition" -> {
                val to = args[0] as Double
                beforeWrite(position, to)
                position = to
                null
            }
            else -> null
        }
    }
}

/** Implements [type] with [call]; anything [call] returns null for gets a default. */
fun <T : Any> deviceProxy(type: Class<T>, call: (String, Array<out Any?>) -> Any?): T =
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
