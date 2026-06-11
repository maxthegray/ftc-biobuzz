package org.firstinspires.ftc.teamcode.core.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.command.Command
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
 *
 * Soft limits ([softMinUnits] / [softMaxUnits]) are enforced in both modes:
 * closed-loop goals are clamped into range, and open-loop power that pushes
 * past a violated limit is zeroed (power away from the limit still works, so
 * a mechanism can always be driven back in bounds).
 */
open class ProfiledMotorSubsystem(
    name: String,
    private val motorName: String,
    protected val controller: ProfiledController,
    private val ticksPerUnit: Double,
    private val direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    private val zeroEncoderOnInit: Boolean = false,
    private val maxOutput: Double = 1.0,
    private val softMinUnits: Double? = null,
    private val softMaxUnits: Double? = null,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase(name) {

    init {
        require(ticksPerUnit != 0.0) { "ticksPerUnit must be non-zero" }
        require(maxOutput > 0.0) { "maxOutput must be positive" }
        if (softMinUnits != null && softMaxUnits != null) {
            require(softMinUnits < softMaxUnits) {
                "softMinUnits ($softMinUnits) must be below softMaxUnits ($softMaxUnits)"
            }
        }
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
            OutputMode.OPEN_LOOP -> limitOpenLoopPower(openLoopPower)
            OutputMode.CLOSED_LOOP -> controller.update(dtSeconds, positionUnits)
        }
        motor.power = power.coerceIn(-maxOutput, maxOutput)
    }

    /** Start (or re-target) closed-loop motion toward [targetUnits], clamped to the soft limits. */
    fun setGoal(targetUnits: Double) {
        if (outputMode != OutputMode.CLOSED_LOOP) {
            controller.reset(positionUnits, velocityUnitsPerSec)
            outputMode = OutputMode.CLOSED_LOOP
        }
        controller.setGoal(clampToSoftLimits(targetUnits))
    }

    private fun clampToSoftLimits(targetUnits: Double): Double {
        var clamped = targetUnits
        softMinUnits?.let { clamped = clamped.coerceAtLeast(it) }
        softMaxUnits?.let { clamped = clamped.coerceAtMost(it) }
        return clamped
    }

    /** Zero power that pushes further past a violated soft limit; power back in bounds passes. */
    private fun limitOpenLoopPower(power: Double): Double {
        val min = softMinUnits
        val max = softMaxUnits
        if (min != null && positionUnits <= min && power < 0.0) return 0.0
        if (max != null && positionUnits >= max && power > 0.0) return 0.0
        return power
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
        .setName("%s→%.1f".format(Locale.US, name, targetUnits))
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
