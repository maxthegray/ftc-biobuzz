package org.firstinspires.ftc.teamcode.core.subsystems

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.control.ProfiledController
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.DeviceReaders
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock

/**
 * A single motor under trapezoid-profiled position control — the
 * season-agnostic base for lifts, arms, and turrets. Extend it (or use it
 * directly) and feed it a [ProfiledController] whose gains/constraints live
 * in a season `@Configurable` object for live tuning.
 *
 * Lifecycle follows the framework contract: [periodic] reads the encoder,
 * [writeHardware] computes and writes power. While in closed-loop mode the
 * controller keeps regulating to the last goal even after a goal command
 * finishes — that is the hold-against-gravity behavior, so no default
 * command is needed.
 *
 * Modes:
 *  - **closed-loop** — entered by [setGoal] / [goToCommand]; profile + PIDF.
 *  - **open-loop** — entered by [openLoop]; raw power for bring-up and
 *    manual override. Exits on the next [setGoal].
 *  - **disabled** — initial state and [disable]; writes zero power.
 *
 * The encoder is *not* reset on init by default ([zeroEncoderOnInit]), so a
 * lift's position survives the auton→teleop op-mode handoff.
 */
open class ProfiledMotorSubsystem(
    name: String,
    private val motorName: String,
    protected val controller: ProfiledController,
    private val ticksPerUnit: Double,
    private val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    private val zeroEncoderOnInit: Boolean = false,
    private val maxOutput: Double = 1.0,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase(name) {

    init {
        require(ticksPerUnit != 0.0) { "ticksPerUnit must be non-zero" }
        require(maxOutput > 0.0) { "maxOutput must be positive" }
    }

    protected enum class OutputMode { DISABLED, OPEN_LOOP, CLOSED_LOOP }

    protected lateinit var motor: DcMotorEx
        private set

    protected var outputMode = OutputMode.DISABLED
        private set

    /** Mechanism position in caller units (encoder ticks ÷ [ticksPerUnit]). */
    var positionUnits: Double = 0.0
        private set

    /** Mechanism velocity in caller units per second. */
    var velocityUnitsPerSec: Double = 0.0
        private set

    private var openLoopPower = 0.0
    private var lastUpdateNs = Long.MIN_VALUE

    override fun init(hardwareMap: HardwareMap) {
        motor = DeviceReaders.motor(
            hardwareMap,
            motorName,
            direction,
            zeroPower = DcMotor.ZeroPowerBehavior.BRAKE,
            runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
        )
        if (zeroEncoderOnInit) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    override fun periodic() {
        positionUnits = motor.currentPosition / ticksPerUnit
        velocityUnitsPerSec = motor.velocity / ticksPerUnit
    }

    override fun writeHardware() {
        val now = clock.nanos()
        val dtSeconds =
            if (lastUpdateNs == Long.MIN_VALUE) 0.0 else (now - lastUpdateNs) / 1e9
        lastUpdateNs = now

        val power = when (outputMode) {
            OutputMode.DISABLED -> 0.0
            OutputMode.OPEN_LOOP -> openLoopPower
            OutputMode.CLOSED_LOOP -> controller.update(dtSeconds, positionUnits)
        }
        motor.power = power.coerceIn(-maxOutput, maxOutput)
    }

    /** Start (or re-target) closed-loop motion toward [targetUnits]. */
    fun setGoal(targetUnits: Double) {
        if (outputMode != OutputMode.CLOSED_LOOP) {
            controller.reset(positionUnits, velocityUnitsPerSec)
            outputMode = OutputMode.CLOSED_LOOP
        }
        controller.setGoal(targetUnits)
    }

    /** Raw power for bring-up / manual override. Sticks until [setGoal] or [disable]. */
    fun openLoop(power: Double) {
        openLoopPower = power
        outputMode = OutputMode.OPEN_LOOP
    }

    /** Cut output. The mechanism coasts/brakes per the motor's zero-power behavior. */
    fun disable() {
        outputMode = OutputMode.DISABLED
    }

    fun atGoal(toleranceUnits: Double): Boolean =
        outputMode == OutputMode.CLOSED_LOOP && controller.atGoal(positionUnits, toleranceUnits)

    /**
     * Command that profiles to [targetUnits] and finishes within
     * [toleranceUnits]. The subsystem keeps holding the goal after the
     * command ends.
     */
    fun goToCommand(
        targetUnits: Double,
        toleranceUnits: Double,
        priority: Int = CommandPriorities.DRIVER_ACTION,
    ): Command = Command.build()
        .requiring(this)
        .setPriority(priority)
        .setStart { setGoal(targetUnits) }
        .setDone { atGoal(toleranceUnits) }

    override fun onCommandFault() {
        // Hold position rather than cut power: a lift dropping under gravity
        // on a contained fault is worse than one frozen in place.
        if (outputMode == OutputMode.OPEN_LOOP) {
            controller.reset(positionUnits, velocityUnitsPerSec)
            controller.setGoal(positionUnits)
            outputMode = OutputMode.CLOSED_LOOP
        }
    }

    override fun health(): String =
        "pos=%.1f vel=%.1f mode=%s".format(Locale.US, positionUnits, velocityUnitsPerSec, outputMode)

    override fun stop() {
        outputMode = OutputMode.DISABLED
        if (::motor.isInitialized) motor.power = 0.0
    }
}
