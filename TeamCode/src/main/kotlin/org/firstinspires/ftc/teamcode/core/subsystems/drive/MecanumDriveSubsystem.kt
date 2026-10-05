package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.algorithm.Foresight
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.math.Velocity
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.hypot
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.runtime.Clock
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
import org.firstinspires.ftc.teamcode.core.subsystems.localization.shortestAngleDelta

/**
 * The one owner of the drivetrain. Every drive command requires this
 * subsystem, so exactly one drives the robot, and every one stops the
 * follower when interrupted (Ivy's own `PedroCommands.follow` doesn't).
 * [Follower.update] runs once per tick, in [writeHardware], after commands
 * have decided what to do.
 *
 * When each command ends:
 *  - [followCommand]: when Pedro reaches the path's parametric end. That is
 *    not arrival; Pedro then holds or idles.
 *  - [holdCommand]: on measured arrival within [DriveConfig] tolerances, or
 *    after its timeout (a bounded wait, not a failure).
 *  - [turnToCommand]: on measured heading; its timeout throws, which aborts
 *    the routine.
 *
 * Each command cleans up at most once per start, because Ivy 1.1.1 can end
 * a command that never started, or end one twice (AI-GUIDE.md).
 */
class MecanumDriveSubsystem(
    val follower: Follower,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Drive"), DriveTelemetrySource {

    data class TeleopInput(
        val forward: Double,
        val strafe: Double,
        val turn: Double,
        val precision: Boolean = false,
        /**
         * When non-null, replaces the curved-and-scaled stick [turn] with a
         * direct motor power, CCW-positive (Pedro's convention, unlike [turn]
         * which is stick-convention +right/CW). For heading assists whose
         * controller output is already a power.
         */
        val turnPower: Double? = null,
        /**
         * When non-null, replaces the curved-and-scaled stick [forward] with a
         * direct motor power (+forward) **and forces robot-centric drive for
         * that tick**: a camera-derived forward means "towards what the camera
         * sees", which is robot-relative by construction.
         */
        val forwardPower: Double? = null,
    )

    /** Per-op-mode driver state, seeded from [DriveConfig.fieldCentricDefault]. */
    var fieldCentric: Boolean = DriveConfig.fieldCentricDefault
        private set

    /**
     * Latched for this op-mode after a localizer fault: manual driving goes
     * straight to the mecanum mixer in the robot frame. No follower update, no
     * odometry read, no field-centric rotation, no path control.
     */
    var odometryFallback: Boolean = false
        private set

    /** Robot-frame powers staged by the teleop command this tick, or null. */
    private var stagedManual: DrivePowers? = null

    private var updateCount = 0L
    private var pathSegments = 1
    private var latchedPathProgress = 0.0

    override fun init(hardwareMap: HardwareMap) {
        loggedMotors = MOTOR_NAMES.map { (label, name) ->
            label to try {
                hardwareMap.tryGet(DcMotorEx::class.java, name)
            } catch (_: Throwable) {
                null
            }
        }.filter { it.second != null }.map { it.first to it.second!! }
    }

    /**
     * Stick-driven manual drive: [DriveConfig] curve, scaling, precision and
     * field-centric selection. Assists reuse it with a higher [priority] and
     * the [TeleopInput.turnPower]/[TeleopInput.forwardPower] overrides.
     */
    fun teleopCommand(
        priority: Int = CommandPriorities.DEFAULT,
        onStart: () -> Unit = {},
        onEnd: (EndCondition) -> Unit = {},
        name: String = "Drive teleop",
        input: () -> TeleopInput,
    ): Command {
        var running = false
        val command = Command.build()
            .requiring(this)
            .setPriority(priority)
            .setStart {
                running = true
                onStart()
            }
            // Ivy still executes a command interrupted earlier in the same tick.
            .setExecute { if (running) stageTeleop(input()) }
            .setDone { false }
            .setEnd { endCondition ->
                if (running) {
                    running = false
                    stagedManual = null
                    follower.stop()
                    onEnd(endCondition)
                }
            }
        return logged(name, command)
    }

    /**
     * Localizer fault recovery: owns drive for the rest of the run at the
     * highest priority, preempting any path or assist, with raw driver sticks
     * in the robot frame. Make it the default command too so nothing reclaims
     * the drive.
     */
    fun robotCentricFallbackCommand(name: String = "Drive robot-centric fallback", input: () -> TeleopInput): Command =
        teleopCommand(priority = Int.MAX_VALUE, onStart = { odometryFallback = true }, name = name, input = input)

    private fun stageTeleop(i: TeleopInput) {
        val scale = DriveConfig.teleopPowerScale *
            (if (i.precision) DriveConfig.precisionPowerScale else 1.0)
        val exp = DriveConfig.inputExponent
        val forward = i.forwardPower?.asDirectPower() ?: (i.forward.curve(exp) * scale)
        // FTC sticks use +x right/CW turn; Pedro uses +lateral left/CCW-positive heading.
        var strafe = -i.strafe.curve(exp) * scale
        var fwd = forward
        // Pedro 2 capped the translation vector at magnitude 1; keep the same driver feel.
        val magnitude = hypot(fwd, strafe)
        if (magnitude > 1.0) {
            fwd /= magnitude
            strafe /= magnitude
        }
        val turn = i.turnPower?.asDirectPower() ?: (-i.turn.curve(exp) * scale)
        val robotCentric = i.forwardPower != null || !fieldCentric || odometryFallback
        // An invalid heading must never rotate the sticks: NaN powers are
        // dropped by Pedro's motor cache, which would leave the last power applied.
        val heading = if (robotCentric) Double.NaN else follower.pose().heading()
        stagedManual = if (robotCentric || !heading.isFinite()) {
            DrivePowers(fwd, strafe, turn)
        } else {
            ManualDrive.fieldCentric(fwd, strafe, turn, heading)
        }
    }

    /**
     * Follow [path] to its parametric end. With [holdEnd] Pedro then holds the
     * end pose; otherwise it idles. Interruption stops the follower.
     */
    fun followCommand(path: Path, holdEnd: Boolean = false, name: String = "Drive follow"): Command {
        var running = false
        val command = Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart {
                running = true
                requirePathControl("follow")
                pathSegments = path.segments.size.coerceAtLeast(1)
                latchedPathProgress = 0.0
                follower.holdEnd.set(holdEnd)
                follower.follow(path)
            }
            .setDone { !follower.following() }
            .setEnd {
                if (running && it != EndCondition.NATURALLY) halt()
                running = false
            }
        return logged(name, command)
    }

    /**
     * Hold [pose] (position and heading) until the measured pose is within
     * [DriveConfig.holdToleranceInches]/[DriveConfig.holdToleranceRadians],
     * or [timeoutMs] passes. Either way the follower keeps holding afterwards;
     * the timeout only bounds how long a routine waits, so FINISH in the log is
     * not proof of arrival.
     */
    fun holdCommand(pose: Pose, timeoutMs: Double = DEFAULT_HOLD_TIMEOUT_MS, name: String = "Drive hold"): Command {
        require(timeoutMs.isFinite() && timeoutMs >= 0.0) { "hold timeout must be finite and non-negative" }
        var updatesAtStart = 0L
        var startNs = 0L
        var running = false
        val command = Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart {
                running = true
                requirePathControl("hold")
                updatesAtStart = updateCount
                startNs = clock.nanos()
                follower.hold(pose)
            }
            .setDone {
                // Wait for one follower update so the measurement postdates the command.
                (updateCount > updatesAtStart && atPose(pose)) || (clock.nanos() - startNs) / 1e6 >= timeoutMs
            }
            .setEnd {
                if (running && it != EndCondition.NATURALLY) halt()
                running = false
            }
        return logged(name, command)
    }

    /**
     * Hold the current position while turning to an absolute heading. Done
     * when the measured heading is within [DriveConfig.holdToleranceRadians].
     * If it has not converged within [timeoutMs] it throws: a timed-out turn is
     * never reported as success. Always stops the follower when it ends.
     */
    fun turnToCommand(radians: Double, timeoutMs: Double = 2_000.0, name: String = "Drive turn"): Command {
        require(radians.isFinite()) { "turn heading must be finite" }
        require(timeoutMs.isFinite() && timeoutMs >= 0.0) { "turn timeout must be finite and non-negative" }
        var startNs = 0L
        var updatesAtStart = 0L
        var running = false
        val command = Command.build()
            .requiring(this)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart {
                running = true
                requirePathControl("turnTo")
                startNs = clock.nanos()
                updatesAtStart = updateCount
                follower.hold(follower.pose().withHeading(radians))
            }
            .setDone {
                val heading = pose.heading()
                check(heading.isFinite()) { "turnTo lost its heading measurement" }
                val error = abs(shortestAngleDelta(heading, radians))
                val reached = updateCount > updatesAtStart && error < DriveConfig.holdToleranceRadians
                check(reached || (clock.nanos() - startNs) / 1e6 < timeoutMs) {
                    "turnTo timed out after $timeoutMs ms; heading error ${Math.toDegrees(error)} deg"
                }
                reached
            }
            .setEnd {
                if (running) halt()
                running = false
            }
        return logged(name, command)
    }

    private fun requirePathControl(action: String) {
        check(follower.algorithm() != null) {
            "$action needs Foresight: tune it with AutoTune and set Constants.FORESIGHT_TUNED"
        }
        check(!odometryFallback) { "$action refused: localizer fault, robot-centric sticks only" }
    }

    override val pose: Pose get() = follower.pose()
    override val velocity: Velocity get() = follower.velocity()

    /** True while Pedro is following a path. */
    val isFollowing: Boolean get() = follower.following()

    /** Whether the measured pose is within the configured hold tolerance of [target]. */
    fun atPose(target: Pose): Boolean {
        val current = pose
        return abs(target.x() - current.x()) < DriveConfig.holdToleranceInches &&
            abs(target.y() - current.y()) < DriveConfig.holdToleranceInches &&
            abs(shortestAngleDelta(current.heading(), target.heading())) < DriveConfig.holdToleranceRadians
    }

    fun toggleFieldCentric() {
        fieldCentric = !fieldCentric
    }

    /**
     * Progress through the path being followed, 0..1 across all of its
     * segments, latched so it never moves backwards. Sampled after each
     * follower update. Reads 0 when nothing is being followed; it reaches 1
     * only when Pedro completes the last segment.
     */
    fun pathProgress(): Double = latchedPathProgress

    // ------------------------------------------------- DriveTelemetrySource

    override val driveModeName: String
        get() = when {
            odometryFallback -> "ROBOT_CENTRIC_FALLBACK"
            else -> when (follower.mode()) {
                Follower.Mode.FOLLOW -> "FOLLOWING"
                Follower.Mode.HOLD -> "HOLDING"
                Follower.Mode.MANUAL -> "TELEOP"
                else -> "IDLE"
            }
        }

    override val isPathing: Boolean
        get() = !odometryFallback && (follower.following() || follower.holding())

    override val followTranslationalErrorInches: Double
        get() = if (!isPathing) Double.NaN else (follower.algorithm() as? Foresight)?.translationalError() ?: Double.NaN

    override val followHeadingErrorRad: Double
        get() = if (!isPathing) Double.NaN else (follower.algorithm() as? Foresight)?.headingError() ?: Double.NaN

    override fun currentPathPoints(samples: Int): List<Vector2D> {
        if (!follower.following()) return emptyList()
        return try {
            val curve = follower.currentPath()?.curve ?: return emptyList()
            (0..samples).map { curve.get(it.toDouble() / samples) }
        } catch (_: Throwable) {
            emptyList()
        }
    }

    // ------------------------------------------------------- motor channels

    private var loggedMotors: List<Pair<String, DcMotorEx>> = emptyList()
    private var motorSampleIndex = 0
    private var lastMotorSampleNs = Long.MIN_VALUE
    private var sampledMotorIndex = -1
    private var sampledPower = 0.0
    private var sampledCurrentAmps = 0.0

    override fun periodic() {
        sampleNextMotor()
    }

    /**
     * getPower()/getCurrent() are real Lynx transactions, not bulk-cache
     * backed, so one motor is sampled every [MOTOR_SAMPLE_INTERVAL_NS]
     * round-robin (each motor refreshes about every 200 ms).
     */
    private fun sampleNextMotor() {
        sampledMotorIndex = -1
        if (loggedMotors.isEmpty()) return
        val now = clock.nanos()
        if (lastMotorSampleNs != Long.MIN_VALUE && now - lastMotorSampleNs < MOTOR_SAMPLE_INTERVAL_NS) return
        lastMotorSampleNs = now
        val index = motorSampleIndex
        motorSampleIndex = (motorSampleIndex + 1) % loggedMotors.size
        try {
            val motor = loggedMotors[index].second
            sampledPower = motor.power
            sampledCurrentAmps = motor.getCurrent(CurrentUnit.AMPS)
            sampledMotorIndex = index
        } catch (_: Throwable) {
            // Logging must never stop the drive; the channel just goes quiet.
        }
    }

    override fun logState(log: StateLog) {
        log.put("fieldCentric", fieldCentric)
        log.put("odometryFallback", odometryFallback)
        val index = sampledMotorIndex
        if (index < 0) return
        val label = loggedMotors[index].first
        log.put("motors/$label/power", sampledPower)
        log.put("motors/$label/currentAmps", sampledCurrentAmps)
    }

    override fun writeHardware() {
        val staged = stagedManual
        stagedManual = null
        if (odometryFallback) {
            follower.drivetrain.drive(staged ?: DrivePowers.zero(), true)
            return
        }
        if (staged != null) follower.manual(staged)
        follower.update()
        updateCount++
        if (follower.following()) {
            val sampled = (follower.pathIndex() + follower.parametricCompletion()) / pathSegments
            if (sampled.isFinite()) latchedPathProgress = maxOf(latchedPathProgress, sampled.coerceIn(0.0, 1.0))
        } else if (!follower.holding()) {
            latchedPathProgress = 0.0
        }
    }

    override fun health(): String =
        if (odometryFallback) "mode=$driveModeName LOCALIZER FAULT: robot-centric sticks only" else "mode=$driveModeName"

    override fun onCommandFault() = halt()

    override fun stop() = halt()

    /** Stop following and write zero power now, without waiting for the next update. */
    private fun halt() {
        stagedManual = null
        latchedPathProgress = 0.0
        follower.stop()
        follower.drivetrain.stop()
    }

    private companion object {
        val MOTOR_NAMES = listOf(
            "leftFront" to RobotConfig.Drive.FRONT_LEFT_MOTOR,
            "leftRear" to RobotConfig.Drive.BACK_LEFT_MOTOR,
            "rightFront" to RobotConfig.Drive.FRONT_RIGHT_MOTOR,
            "rightRear" to RobotConfig.Drive.BACK_RIGHT_MOTOR,
        )
        const val MOTOR_SAMPLE_INTERVAL_NS = 50_000_000L
        const val DEFAULT_HOLD_TIMEOUT_MS = 2_000.0
    }
}

/** A controller output consumed directly as a motor power. A dead controller must not NaN the motors. */
private fun Double.asDirectPower(): Double = if (isFinite()) coerceIn(-1.0, 1.0) else 0.0

/** Applies a signed power curve: preserves sign, scales magnitude by x^exponent. */
private fun Double.curve(exponent: Double): Double = Math.copySign(Math.pow(Math.abs(this), exponent), this)

/**
 * The drive state the flight recorder and the Panels field view consume, so
 * observability code and its tests don't need a real follower. Members must
 * be cheap and exception-free: they run on the hot loop.
 */
interface DriveTelemetrySource {
    /** Field pose: inches, radians, CCW-positive heading, Pedro field frame. */
    val pose: Pose

    /** Field-frame velocity: inches/second and radians/second. */
    val velocity: Velocity

    /** "IDLE", "TELEOP", "FOLLOWING", "HOLDING" or "ROBOT_CENTRIC_FALLBACK". */
    val driveModeName: String

    /** True while following a path or holding a pose (follow errors are live). */
    val isPathing: Boolean

    /** Distance from the closest path point (or hold target) in inches; NaN when unavailable. */
    val followTranslationalErrorInches: Double

    /** Signed heading error in radians; NaN when unavailable. */
    val followHeadingErrorRad: Double

    /** Points along the path being followed, for drawing; empty when not following. */
    fun currentPathPoints(samples: Int): List<Vector2D>
}

/**
 * Drive tuning, live-editable in Panels. Panels edits last until the app
 * restarts or a hot reload; copy values you want to keep into this file.
 * Pedro constants (motor directions, Pinpoint offsets, Foresight) live in
 * [org.firstinspires.ftc.teamcode.pedro.Constants].
 */
@Configurable
object DriveConfig {
    /** Stick curve exponent for forward, strafe and turn: 1 linear, 2 squared, 3 cubic. */
    @JvmField var inputExponent = 2.0

    /** Multiplier on every teleop motion input. */
    @JvmField var teleopPowerScale = 1.0

    /** Multiplier while the precision trigger is held. */
    @JvmField var precisionPowerScale = 0.35

    @JvmField var fieldCentricDefault = true

    /** Brake when the sticks are released. Read when the follower is created, so it applies at the next init. */
    @JvmField var brakeOnTeleop = true

    /** How close a hold or turn must get to its target to count as arrived. */
    @JvmField var holdToleranceInches = 1.0
    @JvmField var holdToleranceRadians = Math.toRadians(2.0)
}
