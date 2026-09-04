package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.follower.Follower
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.Locale
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.Vector2d
import org.firstinspires.ftc.teamcode.core.geometry.shortestAngleDelta
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.pathing.toCore
import org.firstinspires.ftc.teamcode.core.pathing.toPedro
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.DriveTelemetrySource
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.util.Clock
import kotlin.math.abs

/**
 * The one subsystem that owns the mecanum drivetrain. It is a thin façade
 * over the Pedro [Follower] — Pedro already handles motor wiring, field-
 * centric projection, the P/I/D/F loops, and path following. This class
 * exists to:
 *
 *  - Slot into [org.firstinspires.ftc.teamcode.core.runtime.Robot]'s
 *    subsystem lifecycle so [Follower.update] runs at the right moment in
 *    every tick — specifically in [writeHardware], after commands have
 *    decided what the follower should be doing.
 *  - Provide ergonomic `drive(fwd, strafe, turn)` / [followPath] /
 *    [holdPose] methods so op-modes don't have to know about follower
 *    mode toggling.
 *  - Honour [DriveConfig] for teleop scaling, precision mode, and field-
 *    centric selection without a sea of duplicated code in op-modes.
 *
 * Every commanding method is safe to call from inside a command.
 * Conflicting callers should declare `requiring(driveSubsystem)` on their
 * command so the scheduler arbitrates.
 */
class MecanumDriveSubsystem(
    internal val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Drive"), DriveTelemetrySource {

    enum class Mode { IDLE, TELEOP, FOLLOWING, HOLDING }

    data class TeleopInput(
        val forward: Double,
        val strafe: Double,
        val turn: Double,
        val precision: Boolean = false,
        /**
         * When non-null, replaces the curved-and-scaled stick [turn] with a
         * direct motor power, CCW-positive (Pedro's convention, unlike [turn]
         * which is stick-convention +right/CW).
         *
         * For heading assists whose controller output is already a power: the
         * stick input curve squashes small values towards zero, which is
         * exactly where a settling controller lives.
         */
        val turnPower: Double? = null,
        /**
         * When non-null, replaces the curved-and-scaled stick [forward] with a
         * direct motor power (+forward) **and forces robot-centric drive for
         * that tick**.
         *
         * For vision approach assists: a camera-derived forward power means
         * "towards what the camera sees", which is robot-relative by
         * construction. Sending it through the field-centric projection would
         * rotate it into whatever direction the robot happens to be facing.
         * [turnPower] needs no such treatment — turn is frame-independent.
         */
        val forwardPower: Double? = null,
    )

    var mode: Mode = Mode.IDLE
        private set

    /** Per-op-mode driver state, seeded from the persisted tuning default. */
    var fieldCentric: Boolean = DriveConfig.fieldCentricDefault
        private set

    private var modeAfterFollow: Mode = Mode.IDLE
    private var latchedPathProgress = 0.0

    /** Brake-mode argument of a teleop enable deferred to [writeHardware], or null. */
    private var pendingTeleopBrakeMode: Boolean? = null

    /** Ticks of [writeHardware] so far — commands use it to detect "an update has run". */
    private var updateCount = 0L

    override fun init(hardwareMap: HardwareMap) {
        // Follower is constructed in configure() from pedroPathing/Constants;
        // reuse the motors Pedro already resolved for the log channels. A
        // non-Mecanum drivetrain (the sim) just logs nothing.
        loggedMotors = (follower.drivetrain as? Mecanum)?.motors ?: emptyList()
    }

    /**
     * Switch the follower into teleop drive mode. Called by [teleopCommand].
     *
     * The actual [Follower.startTeleOpDrive] call is deferred to
     * [writeHardware]: Pedro 2.1.1 runs a full [Follower.update] inside it,
     * so calling it here (command start) *and* updating again in
     * [writeHardware] would double the localizer read and PID step in one
     * tick. Deferred, it substitutes for that tick's update instead.
     */
    internal fun enableTeleop(brakeMode: Boolean = DriveConfig.brakeOnTeleop) {
        pendingTeleopBrakeMode = brakeMode
        modeAfterFollow = Mode.IDLE
        mode = Mode.TELEOP
    }

    /**
     * Default teleop command. It starts Pedro's teleop drive mode whenever the
     * drive requirement becomes free, then applies [DriveConfig] scaling and
     * field-centric selection every tick.
     *
     * [name] and [priority] exist so a driver assist can reuse this exact path
     * — same input curve, power scale, precision handling, field-centric
     * selection — while overriding individual channels via
     * [TeleopInput.turnPower] / [TeleopInput.forwardPower] and outranking the
     * default command. [onStart] / [onEnd] hang assist-side
     * bookkeeping off the command's lifecycle.
     */
    fun teleopCommand(
        name: String = "teleop drive",
        priority: Int = CommandPriorities.DEFAULT,
        onStart: () -> Unit = {},
        onEnd: (EndCondition) -> Unit = {},
        input: () -> TeleopInput,
    ): Command = Command.build()
        .setName(name)
        .requiring(this)
        .setPriority(priority)
        // onStart composes with the teleop enable rather than replacing it:
        // an assist built on this command must still put the follower into
        // manual-drive mode when it preempts a path.
        .setStart {
            enableTeleop()
            onStart()
        }
        .setExecute {
            val i = input()
            applyTeleopDrive(
                i.forward,
                i.strafe,
                i.turn,
                i.precision,
                i.turnPower,
                i.forwardPower,
            )
        }
        .setDone { false }
        .setEnd(onEnd)

    private fun applyTeleopDrive(
        forward: Double,
        strafe: Double,
        turn: Double,
        precision: Boolean,
        turnPower: Double? = null,
        forwardPower: Double? = null,
    ) {
        val scale = DriveConfig.safeTeleopPowerScale *
            (if (precision) DriveConfig.safePrecisionPowerScale else 1.0)
        val exp = DriveConfig.safeInputExponent
        // The override channels are already motor powers, so they skip the
        // curve — and turnPower skips the stick-convention negation too, being
        // CCW-positive already.
        val fwd = forwardPower?.asDirectPower() ?: (forward.curve(exp) * scale)
        // FTC sticks use +x right/CW turn; Pedro uses +lateral left/CCW-positive heading.
        val strafeScaled = -strafe.curve(exp) * scale
        val turnScaled = turnPower?.asDirectPower() ?: (-turn.curve(exp) * scale)
        // A vision-derived forward is robot-relative by construction — the
        // field-centric projection would rotate it off the target. And a
        // non-finite heading (dead localizer) would NaN that projection and
        // with it the motor powers, so fall back to robot-centric there too and
        // let the driver keep whatever control is still possible.
        val robotCentric =
            forwardPower != null || !fieldCentric || !follower.pose.heading.isFinite()
        follower.setTeleOpDrive(fwd, strafeScaled, turnScaled, robotCentric)
    }

    /**
     * Raw, unscaled drive command for tuning and drivetrain bring-up only.
     *
     * Normal op-modes should use [teleopCommand], which applies driver-feel
     * scaling and participates in the scheduler's requirement arbitration.
     */
    fun driveRaw(forward: Double, strafe: Double, turn: Double, robotCentric: Boolean) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric)
    }

    /** Zero the teleop movement vectors — robot coasts/brakes per [DriveConfig.brakeOnTeleop]. */
    internal fun zero() {
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
    }

    /** Start following a pre-built path chain. */
    internal fun followPath(chain: PathChain, holdEnd: Boolean = true) {
        latchedPathProgress = 0.0
        follower.followPath(chain, holdEnd)
        modeAfterFollow = if (holdEnd) Mode.HOLDING else Mode.IDLE
        mode = Mode.FOLLOWING
    }

    /** Start following a path chain with a custom max-power cap. */
    internal fun followPath(chain: PathChain, maxPower: Double, holdEnd: Boolean = true) {
        latchedPathProgress = 0.0
        follower.followPath(chain, maxPower, holdEnd)
        modeAfterFollow = if (holdEnd) Mode.HOLDING else Mode.IDLE
        mode = Mode.FOLLOWING
    }

    /** Cancel whatever path is running and drift to idle. */
    internal fun breakPath() {
        follower.breakFollowing()
        latchedPathProgress = 0.0
        modeAfterFollow = Mode.IDLE
        mode = Mode.IDLE
    }

    /** Pin the follower to hold a field pose, typically called at the end of an auton leg. */
    internal fun holdPose(pose: Pose2d) {
        latchedPathProgress = 0.0
        follower.holdPoint(pose.toPedro())
        modeAfterFollow = Mode.HOLDING
        mode = Mode.HOLDING
    }

    fun followCommand(chain: PathChain, holdEnd: Boolean = false): Command =
        driveAction(
            name = "follow",
            running = Mode.FOLLOWING,
            finished = if (holdEnd) Mode.HOLDING else Mode.IDLE,
        ) { follower.followPath(chain, holdEnd) }

    fun followCommand(chain: PathChain, maxPower: Double, holdEnd: Boolean = false): Command =
        driveAction(
            name = "follow (maxPower=$maxPower)",
            running = Mode.FOLLOWING,
            finished = if (holdEnd) Mode.HOLDING else Mode.IDLE,
        ) { follower.followPath(chain, maxPower, holdEnd) }

    /**
     * Command that holds [pose] — position *and* heading — via
     * [Follower.holdPoint], so both [holdPose] and this command agree. Done
     * once the follower is within its configured path constraints or
     * [timeoutMs] expires. Either way the subsystem remains in HOLDING mode;
     * the timeout only bounds how long a routine waits for convergence.
     */
    fun holdCommand(pose: Pose2d, timeoutMs: Double = DEFAULT_HOLD_TIMEOUT_MS): Command {
        require(timeoutMs.isFinite() && timeoutMs >= 0.0) {
            "hold timeout must be finite and non-negative"
        }
        var updatesAtStart = 0L
        var startNs = 0L
        return trackDriveMode(
            Command.build()
                .setName("hold (%.1f, %.1f)".format(Locale.US, pose.x, pose.y))
                .setStart {
                    updatesAtStart = updateCount
                    startNs = clock.nanos()
                    follower.holdPoint(pose.toPedro())
                }
                .setDone {
                    // Pedro recomputes its cached errors only inside
                    // Follower.update(); on the tick holdPoint() is issued they
                    // are stale (previous motion's near-zero error would end
                    // this command instantly) or, on a first-ever hold, null
                    // inside Pedro. Wait for one update before evaluating.
                    val reached = updateCount > updatesAtStart &&
                        follower.translationalError.magnitude < follower.constraints.translationalConstraint &&
                        abs(follower.headingError) < follower.constraints.headingConstraint
                    reached || (clock.nanos() - startNs) / 1e6 >= timeoutMs
                }
                .asDriveAction(),
            running = Mode.HOLDING,
            finished = Mode.HOLDING,
        )
    }

    /**
     * Turn in place to an absolute heading: follow a zero-length path pinned at the
     * current pose with constant-heading interpolation at the target, done
     * when the follower stops being busy.
     */
    fun turnToCommand(radians: Double): Command =
        driveAction(
            name = "turnTo %.0f°".format(Locale.US, Math.toDegrees(radians)),
            running = Mode.FOLLOWING,
            finished = Mode.IDLE,
        ) {
            val path = Path(BezierPoint(follower.pose))
            path.setHeadingInterpolation(HeadingInterpolator.constant(radians))
            path.setConstraints(follower.pathConstraints)
            follower.followPath(path)
        }

    /**
     * A drive-claiming command at [CommandPriorities.DRIVER_ACTION]: [start]
     * kicks the follower into a motion, done when the follower goes idle.
     * Interruption (or a fault) breaks the follow so the unconditional
     * `Follower.update()` in [writeHardware] doesn't keep driving the
     * abandoned motion.
     */
    private fun driveAction(
        name: String,
        running: Mode,
        finished: Mode,
        start: () -> Unit,
    ): Command = Command.build()
        .setName(name)
        .requiring(this)
        .setPriority(CommandPriorities.DRIVER_ACTION)
        .setStart {
            modeAfterFollow = finished
            mode = running
            if (running == Mode.FOLLOWING) latchedPathProgress = 0.0
            start()
        }
        .setDone { !follower.isBusy }
        .setEnd { endCondition ->
            if (endCondition == EndCondition.NATURALLY) {
                mode = finished
            } else {
                follower.breakFollowing()
                mode = Mode.IDLE
                modeAfterFollow = Mode.IDLE
            }
        }

    override val pose: Pose2d get() = follower.pose.toCore()
    override val velocity: Vector2d get() = follower.velocity.toCore()

    /** True while Pedro is actively following a path. */
    val isFollowing: Boolean get() = follower.isBusy

    /** True if the robot is actually moving faster than [DriveConfig.stoppedVelocityThreshold]. */
    val isMoving: Boolean
        get() = velocity.magnitude >= DriveConfig.safeStoppedVelocityThreshold

    /** Checks whether the robot is within the configured hold tolerance of a target pose. */
    fun atPose(target: Pose2d): Boolean {
        val current = pose
        return abs(target.x - current.x) < DriveConfig.safeHoldToleranceInches &&
            abs(target.y - current.y) < DriveConfig.safeHoldToleranceInches &&
            abs(shortestAngleDelta(current.heading, target.heading)) <
                DriveConfig.safeHoldToleranceRadians
    }

    fun toggleFieldCentric() {
        fieldCentric = !fieldCentric
    }

    // ------------------------------------------------- DriveTelemetrySource

    override val driveModeName: String get() = mode.name

    override val isPathing: Boolean
        get() = mode == Mode.FOLLOWING || mode == Mode.HOLDING

    override val angularVelocityRadPerSec: Double
        get() = try {
            follower.angularVelocity
        } catch (_: Throwable) {
            Double.NaN
        }

    override val followTranslationalErrorInches: Double
        get() = if (!isPathing) Double.NaN else try {
            follower.translationalError.magnitude
        } catch (_: Throwable) {
            Double.NaN
        }

    override val followHeadingErrorRad: Double
        get() = if (!isPathing) Double.NaN else try {
            follower.headingError
        } catch (_: Throwable) {
            Double.NaN
        }

    /**
     * Progress through the path chain currently being followed, 0..1 across
     * the whole chain (a 3-path chain at the middle of path 2 reads ~0.5).
     * Latches monotonically from Pedro's actual path/t values. Completion
     * never synthesizes 1.0: if Pedro ends a path early, unreached markers
     * remain unfired. Idle/teleop and cancellation read 0.0.
     */
    fun pathProgress(): Double {
        when (mode) {
            Mode.FOLLOWING, Mode.HOLDING -> {}
            else -> return 0.0
        }
        val sampled = try {
            val size = follower.currentPathChain?.size() ?: 1
            (follower.currentPathNumber + follower.currentTValue) / size
        } catch (_: Throwable) {
            Double.NaN
        }
        if (sampled.isFinite()) {
            latchedPathProgress = maxOf(latchedPathProgress, sampled.coerceIn(0.0, 1.0))
        }
        return latchedPathProgress
    }

    override fun currentPathPoses(samplesPerPath: Int): List<List<Pose2d>> {
        if (mode != Mode.FOLLOWING) return emptyList()
        return try {
            val chain = follower.currentPathChain
            val paths: List<Path> = if (chain != null) {
                (0 until chain.size()).map { chain.getPath(it) }
            } else {
                listOfNotNull(follower.currentPath)
            }
            paths.map { path ->
                (0..samplesPerPath).map { i ->
                    path.getPose(i.toDouble() / samplesPerPath).toCore()
                }
            }
        } catch (_: Throwable) {
            emptyList()
        }
    }

    override fun periodic() {
        if (mode == Mode.FOLLOWING && !follower.isBusy) mode = modeAfterFollow
        sampleNextMotor()
    }

    // ------------------------------------------------------- motor channels

    private var loggedMotors: List<DcMotorEx> = emptyList()
    private var motorSampleIndex = 0
    private var lastMotorSampleNs = Long.MIN_VALUE
    private var sampledMotorIndex = -1
    private var sampledPower = 0.0
    private var sampledCurrentAmps = 0.0

    /**
     * getPower()/getCurrent() are real Lynx transactions — not bulk-cache
     * backed — so reading all four motors every tick would cost milliseconds.
     * Instead one motor is sampled per [MOTOR_SAMPLE_INTERVAL_NS] round-robin
     * (each motor refreshes every ~200 ms), spreading the cost evenly.
     */
    private fun sampleNextMotor() {
        sampledMotorIndex = -1
        if (loggedMotors.isEmpty()) return
        val now = clock.nanos()
        if (lastMotorSampleNs != Long.MIN_VALUE && now - lastMotorSampleNs < MOTOR_SAMPLE_INTERVAL_NS) {
            return
        }
        lastMotorSampleNs = now
        val index = motorSampleIndex
        motorSampleIndex = (motorSampleIndex + 1) % loggedMotors.size
        try {
            val motor = loggedMotors[index]
            sampledPower = motor.power
            sampledCurrentAmps = motor.getCurrent(CurrentUnit.AMPS)
            sampledMotorIndex = index
        } catch (_: Throwable) {
            // Logging must never stop the drive; the channel just goes quiet.
        }
    }

    override fun logState(log: StateLog) {
        log.put("fieldCentric", fieldCentric)
        val index = sampledMotorIndex
        if (index < 0) return
        val label = MOTOR_LABELS.getOrElse(index) { "motor$index" }
        log.put("motors/$label/power", sampledPower)
        log.put("motors/$label/currentAmps", sampledCurrentAmps)
    }

    override fun writeHardware() {
        // Pedro's Follower.update() is a single mega-method that reads the
        // localizer, runs the path or teleop vectors, and writes motor powers.
        // Must run every tick, after the command scheduler has decided what
        // setTeleOpDrive / followPath / holdPoint call to issue.
        val pendingBrakeMode = pendingTeleopBrakeMode
        pendingTeleopBrakeMode = null
        if (pendingBrakeMode != null && mode == Mode.TELEOP) {
            // startTeleOpDrive runs Follower.update() internally, so it *is*
            // this tick's update — calling both would double the localizer
            // read and PID step. (The mode guard covers a teleop command
            // preempted by a path in the same tick.)
            follower.startTeleOpDrive(pendingBrakeMode)
        } else {
            follower.update()
        }
        updateCount++
    }

    override fun persistState() {
        PersistedPose.record(pose)
    }

    override fun health(): String = "mode=$mode"

    override fun onCommandFault() = halt()

    override fun stop() = halt()

    private fun halt() {
        zero()
        follower.breakFollowing()
        pendingTeleopBrakeMode = null
        modeAfterFollow = Mode.IDLE
        mode = Mode.IDLE
        latchedPathProgress = 0.0
    }

    /**
     * Wrap an arbitrary drive-touching [command] with this subsystem's
     * requirement and [Mode] bookkeeping. Use it when season code builds its
     * own follower motion; the framework's own commands are built on
     * [driveAction] directly.
     */
    internal fun trackDriveMode(command: Command, running: Mode, finished: Mode): Command =
        Command.build()
            .setName(command.toString())
            .requiring(*(setOf(this) + command.requirements()).toTypedArray())
            .setPriority(command.priority())
            .setStart {
                modeAfterFollow = finished
                mode = running
                if (running == Mode.FOLLOWING) latchedPathProgress = 0.0
                command.start()
            }
            .setExecute(command::execute)
            .setDone(command::done)
            .setEnd { endCondition ->
                command.end(endCondition)
                if (endCondition == EndCondition.NATURALLY) {
                    mode = finished
                } else {
                    // The wrapped command may not break the follow itself —
                    // and update() runs unconditionally in writeHardware, so
                    // an abandoned motion would keep driving.
                    follower.breakFollowing()
                    mode = Mode.IDLE
                    modeAfterFollow = Mode.IDLE
                }
            }

    private companion object {
        /** Pedro's Mecanum.getMotors() order. */
        val MOTOR_LABELS = listOf("leftFront", "leftRear", "rightFront", "rightRear")
        const val MOTOR_SAMPLE_INTERVAL_NS = 50_000_000L
        const val DEFAULT_HOLD_TIMEOUT_MS = 2_000.0
    }
}

private fun CommandBuilder.asDriveAction(): CommandBuilder =
    setPriority(CommandPriorities.DRIVER_ACTION)

/**
 * A controller output consumed directly as a motor power. A dead controller
 * must not NaN the motor powers any more than a dead localizer may.
 */
private fun Double.asDirectPower(): Double = if (isFinite()) coerceIn(-1.0, 1.0) else 0.0

/** Applies a signed power curve: preserves sign, scales magnitude by x^exponent. */
private fun Double.curve(exponent: Double): Double = Math.copySign(Math.pow(Math.abs(this), exponent), this)
