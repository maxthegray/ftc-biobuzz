package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * The middle motors that lift balls to the shooter, and the servo tab
 * (blocker) at the top that keeps them out of the turret. One owner, so the
 * motors never feed into a closed blocker and the blocker never opens without
 * a feed:
 *  - [hold] (the default): blocker closed, motors off.
 *  - [stage]: blocker closed, motors pushing balls up against it.
 *  - [feed]: blocker open; motors run only once it has had
 *    [TransferConfig.blockerTravelMs] to swing clear.
 *  - [reverse]: blocker closed, motors backwards.
 * Every command closes the blocker and stops the motors when it ends.
 *
 * Collecting while staging is `parallel(intake.collect(), transfer.stage())`.
 */
class TransferSubsystem(private val clock: Clock = Clock.SYSTEM) : SubsystemBase("Transfer") {

    private var motors: List<DcMotorEx> = emptyList()
    private var blocker: Servo? = null

    var motorPower: Double = 0.0
        private set

    /** Blocker target; the servo gets it at the next `writeHardware()`. */
    var blockerOpen: Boolean = false
        private set

    private var blockerOpenedAtNs: Long? = null

    /** At least [TransferConfig.blockerTravelMs] since writing the servo's open position. */
    val blockerSettled: Boolean
        get() {
            val openedAtNs = blockerOpenedAtNs ?: return false
            return blockerOpen && (clock.nanos() - openedAtNs) / 1e6 >= TransferConfig.blockerTravelMs
        }

    /** A ball waiting against the blocker; null until a sensor is fitted. */
    val ballStaged: Boolean? = null

    init {
        defaultCommand = hold()
    }

    override fun init(hardwareMap: HardwareMap) {
        motors = RobotConfig.Transfer.MOTORS.map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        blocker = hardwareMap.get(Servo::class.java, RobotConfig.Transfer.BLOCKER_SERVO)
    }

    override fun writeHardware() {
        motors.forEach { it.power = motorPower }
        blocker?.position = if (blockerOpen) TransferConfig.blockerOpenPosition else TransferConfig.blockerClosedPosition
        if (blockerOpen && blockerOpenedAtNs == null) {
            blockerOpenedAtNs = clock.nanos()
        }
    }

    fun hold(): Command = transferCommand("Transfer hold") {
        blockerOpen = false
        motorPower = 0.0
    }

    /** Pushes balls up against the closed blocker until interrupted. */
    fun stage(): Command = transferCommand("Transfer stage") {
        blockerOpen = false
        motorPower = TransferConfig.stagePower
    }

    /** Opens the blocker and, once it is clear, feeds balls into the turret until interrupted. */
    fun feed(): Command = transferCommand("Transfer feed") {
        if (!blockerOpen) {
            blockerOpen = true
            blockerOpenedAtNs = null
        }
        motorPower = if (blockerSettled) TransferConfig.feedPower else 0.0
    }

    /** Runs the middle motors backwards with the blocker closed until interrupted. */
    fun reverse(): Command = transferCommand("Transfer reverse") {
        blockerOpen = false
        motorPower = TransferConfig.reversePower
    }

    private fun transferCommand(name: String, apply: () -> Unit): Command =
        logged(name, infinite { apply() }.requiring(this).setEnd { makeSafe() })

    private fun makeSafe() {
        motorPower = 0.0
        blockerOpen = false
        blockerOpenedAtNs = null
    }

    override fun onCommandFault() = makeSafe()

    override fun stop() {
        makeSafe()
        motors.forEach { it.power = 0.0 }
    }

    override fun logState(log: StateLog) {
        log.put("motorPower", motorPower)
        log.put("blockerOpen", blockerOpen)
        log.put("blockerSettled", blockerSettled)
        log.put("blockerPosition", blocker?.position ?: Double.NaN)
    }

    private companion object {
        /** Motor names whose positive power runs balls down instead of up. */
        val REVERSED = emptySet<String>()
    }
}

/** Transfer motor powers and blocker servo settings, live-editable in Panels. Placeholders until it's built. */
@Configurable
object TransferConfig {
    @JvmField var stagePower = 0.4
    @JvmField var feedPower = 1.0
    @JvmField var reversePower = -0.6
    @JvmField var blockerClosedPosition = 0.0
    @JvmField var blockerOpenPosition = 0.5

    /** How long the blocker takes to swing clear after it is told to open. */
    @JvmField var blockerTravelMs = 150.0
}
