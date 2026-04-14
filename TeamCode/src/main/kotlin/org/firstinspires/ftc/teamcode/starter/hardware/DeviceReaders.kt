package org.firstinspires.ftc.teamcode.starter.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/**
 * Safe, terse hardware lookup. Every helper throws [HardwareConfigError]
 * with the device name when the lookup fails, so wiring typos surface on
 * init instead of as a NullPointerException on the 1000th tick.
 */
object DeviceReaders {

    fun motor(hardwareMap: HardwareMap, name: String): DcMotorEx =
        lookup(hardwareMap, name, DcMotorEx::class.java)

    fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
        zeroPower: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE,
        runMode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
    ): DcMotorEx {
        val m = motor(hardwareMap, name)
        m.direction = direction
        m.zeroPowerBehavior = zeroPower
        m.mode = runMode
        return m
    }

    fun servo(hardwareMap: HardwareMap, name: String): Servo =
        lookup(hardwareMap, name, Servo::class.java)

    fun <T : HardwareDevice> maybe(hardwareMap: HardwareMap, name: String, type: Class<T>): T? =
        try { hardwareMap.get(type, name) } catch (_: Throwable) { null }

    private fun <T : HardwareDevice> lookup(hardwareMap: HardwareMap, name: String, type: Class<T>): T =
        try {
            hardwareMap.get(type, name)
        } catch (t: Throwable) {
            throw HardwareConfigError(
                "Missing ${type.simpleName} named \"$name\" in active configuration.",
                t,
            )
        }
}

class HardwareConfigError(message: String, cause: Throwable? = null) : RuntimeException(message, cause)
