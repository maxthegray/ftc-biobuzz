package org.firstinspires.ftc.teamcode.subsystems.intake

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.io.MotorIO
import org.firstinspires.ftc.teamcode.core.io.RealMotorIO
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase

/**
 * Intake motors ([RobotConfig.Intake.MOTORS]), all driven at one power;
 * positive collects. Collecting while the transfer stages balls is
 * `parallel(intake.collect(), transfer.stage())`, which requires both.
 *
 * Whoever registers it also registers the settings:
 * `ConfigStore.register("intake", IntakeConfig, IntakeConfig::resetDefaults)`.
 */
class IntakeSubsystem(
    private val openMotors: (HardwareMap) -> List<MotorIO> = { realMotors(it) },
) : SubsystemBase("Intake") {

    private var motors: List<MotorIO> = emptyList()

    var power: Double = 0.0
        private set

    override fun init(hardwareMap: HardwareMap) {
        motors = openMotors(hardwareMap)
    }

    override fun writeHardware() {
        motors.forEach { it.setPower(power) }
    }

    /** Runs the intake until interrupted. */
    fun collect(priority: Int = CommandPriorities.DRIVER_ACTION): Command =
        spin("Intake collect", priority) { IntakeConfig.safeCollectPower }

    /** Runs the intake backwards until interrupted, to spit a ball out or clear a jam. */
    fun eject(priority: Int = CommandPriorities.DRIVER_ACTION): Command =
        spin("Intake eject", priority) { IntakeConfig.safeEjectPower }

    private fun spin(name: String, priority: Int, target: () -> Double): Command {
        var running = false
        return logged(
            name,
            Command.build()
                .requiring(this)
                .setPriority(priority)
                .setStart {
                    running = true
                    power = target()
                }
                .setExecute { if (running) power = target() }
                .setDone { false }
                .setEnd {
                    if (running) power = 0.0
                    running = false
                },
        )
    }

    override fun onCommandFault() {
        power = 0.0
    }

    override fun stop() {
        power = 0.0
        motors.forEach { it.setPower(0.0) }
    }

    override fun logState(log: StateLog) {
        log.put("power", power)
    }

    companion object {
        /** Motor names whose positive power runs against collecting. */
        private val REVERSED = emptySet<String>()

        private fun realMotors(hardwareMap: HardwareMap): List<MotorIO> = RobotConfig.Intake.MOTORS.map { name ->
            val direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
            RealMotorIO(DeviceReaders.motor(hardwareMap, name, direction))
        }
    }
}
