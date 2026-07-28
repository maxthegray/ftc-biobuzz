package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import java.util.Locale

/**
 * On-blocks drivetrain mapping check. Dpad left/right selects one motor;
 * the triggers spin only that motor at up to 20% power.
 */
@TeleOp(name = "Starter: Motor Direction Test", group = "Starter")
class MotorDirectionTestTeleOp : OpModeBase() {

    private lateinit var motorTest: DriveMotorTestSubsystem

    override val publishFieldView: Boolean get() = false
    override val endgameRumble: Boolean get() = false

    override fun configure() {
        motorTest = robot.register(DriveMotorTestSubsystem())
        motorTest.defaultCommand = motorTest.testCommand(
            selectionDelta = {
                when {
                    driver.dpadLeftPressed -> -1
                    driver.dpadRightPressed -> 1
                    else -> 0
                }
            },
            requestedPower = {
                (driver.rightTrigger - driver.leftTrigger) * MAX_TEST_POWER
            },
        )
    }

    override fun onLoop() {
        telemetryBag.section("Motor Direction Test") {
            put("SAFETY", "ROBOT ON BLOCKS")
            put("selected", motorTest.selectedLabel)
            put("power", motorTest.requestedPower, decimals = 2)
            put("controls", "dpad L/R select; RT forward; LT reverse")
        }
    }

    private companion object {
        const val MAX_TEST_POWER = 0.2
    }
}

private class DriveMotorTestSubsystem : SubsystemBase("Drive Motor Test") {
    private data class MotorSpec(
        val name: String,
        val direction: DcMotorSimple.Direction,
    )

    private val specs = with(Constants.driveConstants) {
        listOf(
            MotorSpec(leftFrontMotorName, leftFrontMotorDirection),
            MotorSpec(leftRearMotorName, leftRearMotorDirection),
            MotorSpec(rightFrontMotorName, rightFrontMotorDirection),
            MotorSpec(rightRearMotorName, rightRearMotorDirection),
        )
    }

    private lateinit var motors: List<DcMotorEx>
    private var selectedIndex = 0

    var requestedPower: Double = 0.0
        private set

    val selectedLabel: String get() = specs[selectedIndex].name

    override fun init(hardwareMap: HardwareMap) {
        motors = specs.map {
            DeviceReaders.motor(hardwareMap, it.name, it.direction)
        }
    }

    fun testCommand(
        selectionDelta: () -> Int,
        requestedPower: () -> Double,
    ): Command = Command.build()
        .setName("single motor test")
        .requiring(this)
        .setExecute {
            val delta = selectionDelta()
            selectedIndex = (selectedIndex + delta + motors.size) % motors.size
            this.requestedPower = requestedPower().takeIf { it.isFinite() }?.coerceIn(-1.0, 1.0) ?: 0.0
        }
        .setDone { false }
        .setEnd { this.requestedPower = 0.0 }

    override fun writeHardware() {
        for (i in motors.indices) {
            motors[i].power = if (i == selectedIndex) requestedPower else 0.0
        }
    }

    override fun health(): String =
        "$selectedLabel power=${"%.2f".format(Locale.US, requestedPower)}"

    override fun stop() {
        requestedPower = 0.0
        for (motor in motors) {
            try {
                motor.power = 0.0
            } catch (_: Throwable) {
                // Give every motor a chance to stop if one hub write fails.
            }
        }
    }
}
