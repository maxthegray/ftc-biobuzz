package org.firstinspires.ftc.teamcode.core.runtime

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Validates that every device the op-mode needs exists in the active
 * configuration *before* anything tries to resolve it. The worst bring-up
 * experience is a wiring typo surfacing as an opaque crash inside Pedro's
 * FollowerBuilder; this converts it into one message listing every missing
 * device at once.
 *
 * [OpModeBase] runs the check before `configure()`. Op-modes that need more
 * devices override `requiredDevices` and append their own [Requirement]s.
 */
object Preflight {

    data class Requirement(val name: String, val type: Class<out HardwareDevice>)

    /** The devices every op-mode in this template needs: drive + localizer. */
    val standard: List<Requirement> = listOf(
        Requirement(RobotConfig.Drive.FRONT_LEFT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.FRONT_RIGHT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.BACK_LEFT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Drive.BACK_RIGHT_MOTOR, DcMotorEx::class.java),
        Requirement(RobotConfig.Localization.PINPOINT, GoBildaPinpointDriver::class.java),
    )

    /** Throws [HardwareConfigError] listing every missing requirement. */
    fun check(hardwareMap: HardwareMap, requirements: List<Requirement>) {
        val missing = requirements.filter { hardwareMap.tryGet(it.type, it.name) == null }
        if (missing.isEmpty()) return
        val listing = missing.joinToString("\n") { "  - ${it.type.simpleName} \"${it.name}\"" }
        throw HardwareConfigError(
            "Active configuration is missing ${missing.size} device(s):\n$listing",
        )
    }
}
