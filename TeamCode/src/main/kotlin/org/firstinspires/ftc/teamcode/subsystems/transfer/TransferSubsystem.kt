package org.firstinspires.ftc.teamcode.subsystems.transfer

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.io.MotorIO
import org.firstinspires.ftc.teamcode.core.io.RealMotorIO
import org.firstinspires.ftc.teamcode.core.io.RealServoIO
import org.firstinspires.ftc.teamcode.core.io.ServoIO
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

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
 * Whoever registers it also registers the settings:
 * `ConfigStore.register("transfer", TransferConfig, TransferConfig::resetDefaults)`.
 */
class TransferSubsystem(
    private val clock: Clock = Clock.SYSTEM,
    private val openHardware: (HardwareMap) -> Hardware = { realHardware(it) },
) : SubsystemBase("Transfer") {

    class Hardware(val motors: List<MotorIO>, val blocker: ServoIO)

    private var hardware: Hardware? = null

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
            return blockerOpen && (clock.nanos() - openedAtNs) / 1e6 >= TransferConfig.safeBlockerTravelMs
        }

    /** A ball waiting against the blocker; null until a sensor is fitted. */
    val ballStaged: Boolean? = null

    init {
        defaultCommand = hold()
    }

    override fun init(hardwareMap: HardwareMap) {
        hardware = openHardware(hardwareMap)
    }

    override fun writeHardware() {
        val hw = hardware ?: return
        hw.motors.forEach { it.setPower(motorPower) }
        hw.blocker.setPosition(
            if (blockerOpen) TransferConfig.safeBlockerOpenPosition else TransferConfig.safeBlockerClosedPosition,
        )
        if (blockerOpen && blockerOpenedAtNs == null) {
            blockerOpenedAtNs = clock.nanos()
        }
    }

    fun hold(): Command = transferCommand("Transfer hold", CommandPriorities.DEFAULT) {
        blockerOpen = false
        motorPower = 0.0
    }

    /** Pushes balls up against the closed blocker until interrupted. */
    fun stage(priority: Int = CommandPriorities.DRIVER_ACTION): Command = transferCommand("Transfer stage", priority) {
        blockerOpen = false
        motorPower = TransferConfig.safeStagePower
    }

    /** Opens the blocker and, once it is clear, feeds balls into the turret until interrupted. */
    fun feed(priority: Int = CommandPriorities.DRIVER_ACTION): Command = transferCommand("Transfer feed", priority) {
        if (!blockerOpen) {
            blockerOpen = true
            blockerOpenedAtNs = null
        }
        motorPower = if (blockerSettled) TransferConfig.safeFeedPower else 0.0
    }

    /** Runs the middle motors backwards with the blocker closed until interrupted. */
    fun reverse(priority: Int = CommandPriorities.DRIVER_ACTION): Command = transferCommand("Transfer reverse", priority) {
        blockerOpen = false
        motorPower = TransferConfig.safeReversePower
    }

    private fun transferCommand(name: String, priority: Int, apply: () -> Unit): Command {
        var running = false
        return logged(
            name,
            Command.build()
                .requiring(this)
                .setPriority(priority)
                .setStart {
                    running = true
                    apply()
                }
                .setExecute { if (running) apply() }
                .setDone { false }
                .setEnd {
                    if (running) makeSafe()
                    running = false
                },
        )
    }

    private fun makeSafe() {
        motorPower = 0.0
        blockerOpen = false
        blockerOpenedAtNs = null
    }

    override fun onCommandFault() = makeSafe()

    override fun stop() {
        makeSafe()
        hardware?.motors?.forEach { it.setPower(0.0) }
    }

    override fun logState(log: StateLog) {
        log.put("motorPower", motorPower)
        log.put("blockerOpen", blockerOpen)
        log.put("blockerSettled", blockerSettled)
        log.put("blockerPosition", hardware?.blocker?.lastPosition ?: Double.NaN)
    }

    companion object {
        /** Motor names whose positive power runs balls down instead of up. */
        private val REVERSED = emptySet<String>()

        private fun realHardware(hardwareMap: HardwareMap) = Hardware(
            motors = RobotConfig.Transfer.MOTORS.map { name ->
                val motorDirection = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                RealMotorIO(hardwareMap.get(DcMotorEx::class.java, name).apply {
                    direction = motorDirection
                    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                    mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                })
            },
            blocker = RealServoIO(hardwareMap.get(Servo::class.java, RobotConfig.Transfer.BLOCKER_SERVO)),
        )
    }
}

/**
 * Middle-motor powers and blocker servo settings, persisted under `transfer`.
 * Placeholders until the transfer is built: tune in Panels.
 */
@Configurable
object TransferConfig {

    private const val DEFAULT_STAGE_POWER = 0.4
    private const val DEFAULT_FEED_POWER = 1.0
    private const val DEFAULT_REVERSE_POWER = -0.6
    private const val DEFAULT_BLOCKER_CLOSED_POSITION = 0.0
    private const val DEFAULT_BLOCKER_OPEN_POSITION = 0.5
    private const val DEFAULT_BLOCKER_TRAVEL_MS = 150.0

    /** Pushes balls up against the closed blocker. */
    @JvmField var stagePower: Double = DEFAULT_STAGE_POWER

    /** Pushes balls past the open blocker into the turret. */
    @JvmField var feedPower: Double = DEFAULT_FEED_POWER

    @JvmField var reversePower: Double = DEFAULT_REVERSE_POWER

    @JvmField var blockerClosedPosition: Double = DEFAULT_BLOCKER_CLOSED_POSITION
    @JvmField var blockerOpenPosition: Double = DEFAULT_BLOCKER_OPEN_POSITION

    /** Time for the blocker to swing open; a feed runs no motors before it has passed. */
    @JvmField var blockerTravelMs: Double = DEFAULT_BLOCKER_TRAVEL_MS

    fun resetDefaults() {
        stagePower = DEFAULT_STAGE_POWER
        feedPower = DEFAULT_FEED_POWER
        reversePower = DEFAULT_REVERSE_POWER
        blockerClosedPosition = DEFAULT_BLOCKER_CLOSED_POSITION
        blockerOpenPosition = DEFAULT_BLOCKER_OPEN_POSITION
        blockerTravelMs = DEFAULT_BLOCKER_TRAVEL_MS
    }

    internal val safeStagePower: Double get() = power(stagePower, DEFAULT_STAGE_POWER)
    internal val safeFeedPower: Double get() = power(feedPower, DEFAULT_FEED_POWER)
    internal val safeReversePower: Double get() = power(reversePower, DEFAULT_REVERSE_POWER)
    internal val safeBlockerClosedPosition: Double get() = position(blockerClosedPosition, DEFAULT_BLOCKER_CLOSED_POSITION)
    internal val safeBlockerOpenPosition: Double get() = position(blockerOpenPosition, DEFAULT_BLOCKER_OPEN_POSITION)
    internal val safeBlockerTravelMs: Double
        get() = if (blockerTravelMs.isFinite() && blockerTravelMs >= 0.0) blockerTravelMs else DEFAULT_BLOCKER_TRAVEL_MS

    private fun power(value: Double, default: Double) = if (value.isFinite()) value.coerceIn(-1.0, 1.0) else default
    private fun position(value: Double, default: Double) = if (value.isFinite()) value.coerceIn(0.0, 1.0) else default
}
