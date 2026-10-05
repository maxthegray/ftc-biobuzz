package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * Intake motors ([RobotConfig.Intake.MOTORS]), all driven at one power;
 * positive collects. Collecting while the transfer stages balls is
 * `parallel(intake.collect(), transfer.stage())`, which requires both.
 */
class IntakeSubsystem : SubsystemBase("Intake") {

    private var motors: List<DcMotorEx> = emptyList()

    var power: Double = 0.0
        private set

    override fun init(hardwareMap: HardwareMap) {
        motors = RobotConfig.Intake.MOTORS.map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
    }

    override fun writeHardware() {
        motors.forEach { it.power = power }
    }

    /** Runs the intake until interrupted. */
    fun collect(): Command = spin("Intake collect") { IntakeConfig.collectPower }

    /** Runs the intake backwards until interrupted, to spit a ball out or clear a jam. */
    fun eject(): Command = spin("Intake eject") { IntakeConfig.ejectPower }

    private fun spin(name: String, target: () -> Double): Command =
        logged(name, infinite { power = target() }.requiring(this).setEnd { power = 0.0 })

    override fun onCommandFault() {
        power = 0.0
    }

    override fun stop() {
        power = 0.0
        motors.forEach { it.power = 0.0 }
    }

    override fun logState(log: StateLog) {
        log.put("power", power)
    }

    private companion object {
        /** Motor names whose positive power runs against collecting. */
        val REVERSED = emptySet<String>()
    }
}

/** Intake powers, live-editable in Panels. Placeholders until the intake is built. */
@Configurable
object IntakeConfig {
    @JvmField var collectPower = 1.0
    @JvmField var ejectPower = -0.6
}
