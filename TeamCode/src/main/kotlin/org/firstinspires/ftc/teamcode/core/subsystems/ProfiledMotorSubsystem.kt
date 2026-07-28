package org.firstinspires.ftc.teamcode.core.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.control.ProfiledController
import org.firstinspires.ftc.teamcode.core.hw.MotorIO
import org.firstinspires.ftc.teamcode.core.hw.RealMotorIO
import org.firstinspires.ftc.teamcode.core.logging.StateLog
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
 * Hardware access goes through [MotorIO]: real op-modes resolve a
 * [RealMotorIO] from the hardware map in [init]; host tests inject a
 * `SimMotorIO` via the [io] constructor parameter and the whole subsystem —
 * profile, PIDF, soft limits, homing — runs headless.
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
 * A fresh mechanism is [HomingState.UNHOMED]: only open-loop movement,
 * [setCurrentPosition], and [homeCommand] are allowed. Coordinate-based
 * closed-loop goals and soft limits remain disabled until a zero is known.
 * [persistState] carries a successfully established software zero into the
 * next op-mode instance in the same Robot Controller process.
 *
 * Soft limits ([softMinUnits] / [softMaxUnits]) are enforced in both modes:
 * closed-loop goals are clamped into range, and open-loop power that pushes
 * past a violated limit is zeroed (power away from the limit still works, so
 * a mechanism can always be driven back in bounds). They assume positive
 * power moves toward +units.
 *
 * The encoder is *not* reset on init by default ([zeroEncoderOnInit]), so a
 * lift's position survives the auton→teleop op-mode handoff. [homeCommand]
 * re-establishes a true zero by driving into the hard stop.
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
    io: MotorIO? = null,
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

    protected enum class OutputMode { DISABLED, OPEN_LOOP, CLOSED_LOOP, HOMING }

    enum class HomingState { UNHOMED, HOMING, HOMED, FAULTED }

    enum class GoalOutcome { IDLE, RUNNING, REACHED, TIMED_OUT, INTERRUPTED, FAULTED }

    enum class GoalTimeoutPolicy { HOLD_GOAL, DISABLE }

    class HomingTimeoutException(message: String) : IllegalStateException(message)

    private var injectedIo: MotorIO? = io

    protected lateinit var io: MotorIO
        private set

    protected var outputMode = OutputMode.DISABLED
        private set

    var homingState: HomingState = HomingState.UNHOMED
        private set

    var lastGoalOutcome: GoalOutcome = GoalOutcome.IDLE
        private set

    /** Mechanism position in caller units (encoder ticks ÷ [ticksPerUnit] + home offset). */
    var positionUnits: Double = 0.0
        private set

    /** Mechanism velocity in caller units per second. */
    var velocityUnitsPerSec: Double = 0.0
        private set

    private var offsetUnits = 0.0
    private var openLoopPower = 0.0
    private var lastUpdateNs = Long.MIN_VALUE

    override fun init(hardwareMap: HardwareMap) {
        io = injectedIo ?: RealMotorIO(
            DeviceReaders.motor(
                hardwareMap,
                motorName,
                direction,
                zeroPower = DcMotor.ZeroPowerBehavior.BRAKE,
                runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            ),
        )
        if (zeroEncoderOnInit) {
            io.resetEncoder()
            offsetUnits = 0.0
            homingState = HomingState.HOMED
        } else {
            val handoff = persistedHoming[motorName]
            if (handoff != null && handoff.ticksPerUnit == ticksPerUnit &&
                handoff.offsetUnits.isFinite()
            ) {
                offsetUnits = handoff.offsetUnits
                homingState = HomingState.HOMED
            }
        }
    }

    override fun periodic() {
        positionUnits = io.positionTicks / ticksPerUnit + offsetUnits
        velocityUnitsPerSec = io.velocityTicksPerSec / ticksPerUnit
    }

    override fun writeHardware() {
        val now = clock.nanos()
        val dtSeconds =
            if (lastUpdateNs == Long.MIN_VALUE) 0.0 else (now - lastUpdateNs) / 1e9
        lastUpdateNs = now

        val requestedPower = when (outputMode) {
            OutputMode.DISABLED -> 0.0
            OutputMode.OPEN_LOOP ->
                if (homingState == HomingState.HOMED) limitOpenLoopPower(openLoopPower)
                else openLoopPower
            // Homing bypasses soft limits on purpose — it is looking for the
            // hard stop, and the pre-homing zero may be wrong anyway.
            OutputMode.HOMING -> openLoopPower
            OutputMode.CLOSED_LOOP -> controller.update(dtSeconds, positionUnits)
        }
        val power =
            if (requestedPower.isFinite()) {
                requestedPower.coerceIn(-maxOutput, maxOutput)
            } else {
                homingState = HomingState.FAULTED
                outputMode = OutputMode.DISABLED
                0.0
            }
        io.setPower(power)
    }

    /**
     * Start (or re-target) closed-loop motion toward [targetUnits], clamped
     * to the soft limits. This imperative entry point bypasses scheduler
     * arbitration; prefer [goToCommand] outside a command/marker callback.
     */
    fun setGoal(targetUnits: Double) {
        check(homingState == HomingState.HOMED) {
            "$name cannot use closed-loop goals while $homingState; home or set its position first"
        }
        if (outputMode != OutputMode.CLOSED_LOOP) {
            controller.reset(positionUnits, velocityUnitsPerSec)
            outputMode = OutputMode.CLOSED_LOOP
        }
        controller.setGoal(clampToSoftLimits(targetUnits))
    }

    /**
     * Raw power for bring-up, homing preparation, or manual override. This
     * bypasses scheduler arbitration and, while unhomed, bypasses soft limits
     * because their coordinate frame is not yet valid.
     */
    fun openLoop(power: Double) {
        openLoopPower = power
        outputMode = OutputMode.OPEN_LOOP
    }

    /** Cut output immediately, bypassing scheduler arbitration. */
    fun disable() {
        outputMode = OutputMode.DISABLED
    }

    /**
     * Redefine the current physical position as [units] (software offset —
     * the raw encoder is untouched, so the auton→teleop handoff still works).
     */
    fun setCurrentPosition(units: Double) {
        require(units.isFinite()) { "current position must be finite" }
        offsetUnits = units - io.positionTicks / ticksPerUnit
        positionUnits = units
        homingState = HomingState.HOMED
    }

    fun atGoal(toleranceUnits: Double): Boolean =
        outputMode == OutputMode.CLOSED_LOOP && controller.atGoal(positionUnits, toleranceUnits)

    /**
     * Command that profiles to [targetUnits] and finishes within
     * [toleranceUnits]. The subsystem keeps holding the goal after the
     * command ends.
     *
     * [timeoutMs] bounds the wait: a mechanism that stalls short of the goal
     * (jammed, blocked, mistuned) otherwise hangs an auton step forever. On
     * timeout the command completes normally and the subsystem keeps holding
     * the unreached goal — the safe default for a lift under gravity. Read
     * [lastGoalOutcome] to distinguish a reached goal from a timeout. Use
     * [GoalTimeoutPolicy.DISABLE] when continuing to push is less safe.
     */
    fun goToCommand(
        targetUnits: Double,
        toleranceUnits: Double,
        priority: Int = CommandPriorities.DRIVER_ACTION,
        timeoutMs: Double? = null,
        timeoutPolicy: GoalTimeoutPolicy = GoalTimeoutPolicy.HOLD_GOAL,
    ): Command {
        if (timeoutMs != null) {
            require(timeoutMs.isFinite() && timeoutMs >= 0.0) {
                "timeoutMs must be finite and non-negative"
            }
        }
        var startNs = 0L
        return Command.build()
            .setName("%s→%.1f".format(Locale.US, name, targetUnits))
            .requiring(this)
            .setPriority(priority)
            .setStart {
                startNs = clock.nanos()
                lastGoalOutcome = GoalOutcome.RUNNING
                setGoal(targetUnits)
            }
            .setDone {
                when {
                    atGoal(toleranceUnits) -> {
                        lastGoalOutcome = GoalOutcome.REACHED
                        true
                    }
                    timeoutMs != null && (clock.nanos() - startNs) / 1e6 >= timeoutMs -> {
                        lastGoalOutcome = GoalOutcome.TIMED_OUT
                        true
                    }
                    else -> false
                }
            }
            .setEnd { condition ->
                when (condition) {
                    EndCondition.NATURALLY -> {
                        if (
                            lastGoalOutcome == GoalOutcome.TIMED_OUT &&
                            timeoutPolicy == GoalTimeoutPolicy.DISABLE
                        ) {
                            disable()
                        }
                    }
                    EndCondition.INTERRUPTED -> lastGoalOutcome = GoalOutcome.INTERRUPTED
                    EndCondition.FAULTED -> lastGoalOutcome = GoalOutcome.FAULTED
                }
            }
    }

    /**
     * Homing routine: drive open-loop at [power] into the hard stop, detect
     * the stall (|velocity| below [stallVelocityUnitsPerSec] for
     * [stallTimeMs], checked only after [graceMs] so the spin-up doesn't
     * read as a stall), then declare the current position [resetToUnits] and
     * hold there closed-loop. An interrupted re-home keeps its prior valid
     * frame; an interrupted first home disables output and stays unhomed.
     * [timeoutMs] is mandatory and faults the command if no stall is detected;
     * a timeout never declares an arbitrary position to be zero.
     *
     * Soft limits are bypassed while homing — the entire point is to find
     * the hard stop, and the pre-homing zero may be wrong anyway.
     */
    fun homeCommand(
        power: Double,
        stallVelocityUnitsPerSec: Double,
        timeoutMs: Double,
        stallTimeMs: Double = 150.0,
        graceMs: Double = 250.0,
        resetToUnits: Double = 0.0,
        priority: Int = CommandPriorities.DRIVER_ACTION,
    ): Command {
        require(power != 0.0) { "homing needs a non-zero drive power" }
        require(stallVelocityUnitsPerSec > 0.0) { "stall threshold must be positive" }
        require(stallTimeMs.isFinite() && stallTimeMs >= 0.0) {
            "stallTimeMs must be finite and non-negative"
        }
        require(graceMs.isFinite() && graceMs >= 0.0) {
            "graceMs must be finite and non-negative"
        }
        require(timeoutMs.isFinite() && timeoutMs > graceMs + stallTimeMs) {
            "timeoutMs must be finite and longer than graceMs + stallTimeMs"
        }
        var startNs = 0L
        var stalledSinceNs = Long.MIN_VALUE
        var stateBeforeHome = HomingState.UNHOMED
        return Command.build()
            .setName("$name home")
            .requiring(this)
            .setPriority(priority)
            .setStart {
                check(homingState != HomingState.HOMING) { "$name is already homing" }
                startNs = clock.nanos()
                stalledSinceNs = Long.MIN_VALUE
                stateBeforeHome = homingState
                homingState = HomingState.HOMING
                openLoopPower = power
                outputMode = OutputMode.HOMING
            }
            .setExecute {
                val now = clock.nanos()
                if ((now - startNs) / 1e6 < graceMs) return@setExecute
                if (abs(velocityUnitsPerSec) < stallVelocityUnitsPerSec) {
                    if (stalledSinceNs == Long.MIN_VALUE) stalledSinceNs = now
                } else {
                    stalledSinceNs = Long.MIN_VALUE
                }
            }
            .setDone {
                val now = clock.nanos()
                if (
                    stalledSinceNs != Long.MIN_VALUE &&
                    (now - stalledSinceNs) / 1e6 >= stallTimeMs
                ) {
                    true
                } else if ((now - startNs) / 1e6 >= timeoutMs) {
                    homingState = HomingState.FAULTED
                    throw HomingTimeoutException("$name homing timed out after $timeoutMs ms")
                } else {
                    false
                }
            }
            .setEnd { condition ->
                if (condition == EndCondition.NATURALLY) {
                    setCurrentPosition(resetToUnits)
                    controller.reset(positionUnits, 0.0)
                    controller.setGoal(positionUnits)
                    outputMode = OutputMode.CLOSED_LOOP
                } else if (
                    condition == EndCondition.INTERRUPTED &&
                    stateBeforeHome == HomingState.HOMED
                ) {
                    homingState = HomingState.HOMED
                    controller.reset(positionUnits, velocityUnitsPerSec)
                    controller.setGoal(clampToSoftLimits(positionUnits))
                    outputMode = OutputMode.CLOSED_LOOP
                } else {
                    homingState =
                        if (condition == EndCondition.FAULTED) {
                            HomingState.FAULTED
                        } else {
                            stateBeforeHome
                        }
                    outputMode = OutputMode.DISABLED
                }
            }
    }

    override fun onCommandFault() {
        // Hold position rather than cut power: a lift dropping under gravity
        // on a contained fault is worse than one frozen in place. That
        // includes closed-loop — the faulting command's last goal may be far
        // away, and "frozen" means here, not there. A DISABLED mechanism
        // stays disabled — don't energize something deliberately off.
        if (homingState != HomingState.HOMED) {
            if (homingState == HomingState.HOMING) homingState = HomingState.FAULTED
            outputMode = OutputMode.DISABLED
            return
        }
        if (outputMode == OutputMode.DISABLED) return
        controller.reset(positionUnits, velocityUnitsPerSec)
        controller.setGoal(clampToSoftLimits(positionUnits))
        outputMode = OutputMode.CLOSED_LOOP
    }

    override fun health(): String =
        "pos=%.1f vel=%.1f mode=%s home=%s".format(
            Locale.US,
            positionUnits,
            velocityUnitsPerSec,
            outputMode,
            homingState,
        )

    override fun logState(log: StateLog) {
        log.put("positionUnits", positionUnits)
        log.put("velocityUnitsPerSec", velocityUnitsPerSec)
        log.put("outputPower", if (::io.isInitialized) io.lastPower else 0.0)
        log.put("mode", outputMode.name)
        log.put("homingState", homingState.name)
        log.put("goalOutcome", lastGoalOutcome.name)
        if (outputMode == OutputMode.CLOSED_LOOP) {
            log.put("goalUnits", controller.goal.position)
            log.put("setpointUnits", controller.setpoint.position)
            log.put("setpointVelocity", controller.setpoint.velocity)
        }
    }

    override fun stop() {
        outputMode = OutputMode.DISABLED
        if (::io.isInitialized) io.setPower(0.0)
    }

    override fun persistState() {
        if (homingState == HomingState.HOMED && offsetUnits.isFinite()) {
            persistedHoming[motorName] = HomingHandoff(ticksPerUnit, offsetUnits)
        }
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

    private data class HomingHandoff(val ticksPerUnit: Double, val offsetUnits: Double)

    companion object {
        private val persistedHoming = HashMap<String, HomingHandoff>()

        internal fun clearPersistedHomingForTest() {
            persistedHoming.clear()
        }
    }
}
