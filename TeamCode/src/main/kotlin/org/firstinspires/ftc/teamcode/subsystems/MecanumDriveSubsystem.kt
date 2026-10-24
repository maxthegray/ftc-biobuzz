package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.algorithm.Foresight
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.math.Velocity
import com.pedropathing.paths.Path
import com.pedropathing.revhub.drivetrains.Mecanum
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.hypot
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.pedro.VoltageCompensatedMecanum

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
 *  - [turnToCommand]: on measured heading, or after its timeout (a bounded
 *    wait, so the routine moves on).
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
        motors = MOTOR_NAMES.map { (_, name) ->
            try {
                hardwareMap.tryGet(DcMotorEx::class.java, name)
            } catch (_: Throwable) {
                null
            }
        }
    }

    /** Stick-driven manual drive: [DriveConfig] curve, scaling, precision and field-centric selection. */
    fun teleopCommand(name: String = "Drive teleop", input: () -> TeleopInput): Command =
        logged(name, teleop(onStart = {}, input = input))

    private fun teleop(onStart: () -> Unit, input: () -> TeleopInput): CommandBuilder {
        var running = false
        return Command.build()
            .requiring(this)
            .setStart {
                running = true
                onStart()
            }
            // Ivy still executes a command interrupted earlier in the same tick.
            .setExecute { if (running) stageTeleop(input()) }
            .setEnd {
                if (running) {
                    running = false
                    stagedManual = null
                    follower.stop()
                }
            }
    }

    /**
     * Localizer fault recovery: raw driver sticks in the robot frame for the
     * rest of the run. It is the one command with a priority: the highest, so
     * it replaces any path or assist and nothing can take the drive back.
     * Make it the default command too.
     */
    fun robotCentricFallbackCommand(name: String = "Drive robot-centric fallback", input: () -> TeleopInput): Command =
        logged(name, teleop(onStart = { odometryFallback = true }, input = input).setPriority(Int.MAX_VALUE))

    private fun stageTeleop(i: TeleopInput) {
        val scale = DriveConfig.teleopPowerScale *
            (if (i.precision) DriveConfig.precisionPowerScale else 1.0)
        val exp = DriveConfig.inputExponent.takeIf { it.isFinite() && it > 0.0 } ?: 1.0
        var fwd = i.forward.curve(exp) * scale
        // FTC sticks use +x right/CW turn; Pedro uses +lateral left/CCW-positive heading.
        var strafe = -i.strafe.curve(exp) * scale
        // Pedro 2 capped the translation vector at magnitude 1; keep the same driver feel.
        val magnitude = hypot(fwd, strafe)
        if (magnitude > 1.0) {
            fwd /= magnitude
            strafe /= magnitude
        }
        val turn = -i.turn.curve(exp) * scale
        val robotCentric = !fieldCentric || odometryFallback
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
    fun holdCommand(pose: Pose, timeoutMs: Double = DEFAULT_TIMEOUT_MS, name: String = "Drive hold"): Command {
        require(timeoutMs.isFinite() && timeoutMs >= 0.0) { "hold timeout must be finite and non-negative" }
        var updatesAtStart = 0L
        var startNs = 0L
        var running = false
        val command = Command.build()
            .requiring(this)
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
     * when the measured heading is within [DriveConfig.holdToleranceRadians],
     * or after [timeoutMs], so a turn that can't quite converge doesn't stall
     * the routine; FINISH in the log is not proof it got there. Always stops
     * the follower when it ends.
     */
    fun turnToCommand(radians: Double, timeoutMs: Double = DEFAULT_TIMEOUT_MS, name: String = "Drive turn"): Command {
        require(radians.isFinite()) { "turn heading must be finite" }
        require(timeoutMs.isFinite() && timeoutMs >= 0.0) { "turn timeout must be finite and non-negative" }
        var startNs = 0L
        var updatesAtStart = 0L
        var running = false
        val command = Command.build()
            .requiring(this)
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
                val reached = updateCount > updatesAtStart &&
                    abs(shortestAngleDelta(heading, radians)) < DriveConfig.holdToleranceRadians
                reached || (clock.nanos() - startNs) / 1e6 >= timeoutMs
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

    /** In Pedro's mixer order ([MOTOR_NAMES]); null where the hardware map has no such motor. */
    private var motors: List<DcMotorEx?> = emptyList()
    private val mecanum = follower.drivetrain as? Mecanum
    private val commandedPower = DoubleArray(MOTOR_NAMES.size)
    private val wheelVelocity = DoubleArray(MOTOR_NAMES.size) { Double.NaN }
    private var currentSampleIndex = 0
    private var lastCurrentSampleNs = Long.MIN_VALUE
    private var sampledCurrentIndex = -1
    private var sampledCurrentAmps = 0.0

    override fun periodic() {
        readWheelVelocities()
        sampleNextCurrent()
    }

    /** Encoder velocities come from the bulk read, so every tick is free. */
    private fun readWheelVelocities() {
        for (i in wheelVelocity.indices) {
            wheelVelocity[i] = try {
                motors.getOrNull(i)?.velocity ?: Double.NaN
            } catch (_: Throwable) {
                Double.NaN
            }
        }
    }

    /**
     * getCurrent() is a real Lynx transaction, not bulk-cache backed, so one
     * motor is sampled every [CURRENT_SAMPLE_INTERVAL_NS] round-robin (each
     * motor refreshes about every 800 ms). The battery analysis uses the hubs'
     * total current instead; this only shows which motor draws what. A sample
     * stays pending until [logState] writes it, since the recorder skips ticks.
     */
    private fun sampleNextCurrent() {
        if (motors.isEmpty()) return
        val now = clock.nanos()
        if (lastCurrentSampleNs != Long.MIN_VALUE && now - lastCurrentSampleNs < CURRENT_SAMPLE_INTERVAL_NS) return
        lastCurrentSampleNs = now
        val index = currentSampleIndex
        currentSampleIndex = (currentSampleIndex + 1) % motors.size
        val motor = motors[index] ?: return
        try {
            sampledCurrentAmps = motor.getCurrent(CurrentUnit.AMPS)
            sampledCurrentIndex = index
        } catch (_: Throwable) {
            // Logging must never stop the drive; the channel just goes quiet.
        }
    }

    /**
     * Pedro's mixer keeps the last powers it wrote, but its stop() zeroes the
     * motors without touching them, so a stopped drivetrain is recorded as zero.
     */
    private fun recordCommandedPower(stopped: Boolean) {
        val wheels = mecanum?.wheelPowers
        for (i in commandedPower.indices) {
            commandedPower[i] = when {
                wheels == null -> Double.NaN
                stopped -> 0.0
                else -> wheels[i]
            }
        }
    }

    override fun logState(log: StateLog) {
        log.put("fieldCentric", fieldCentric)
        log.put("odometryFallback", odometryFallback)
        log.put("motors/power", commandedPower)
        log.put("motors/velocityTps", wheelVelocity)
        (follower.drivetrain as? VoltageCompensatedMecanum)?.let { log.put("voltageScale", it.scale()) }
        val index = sampledCurrentIndex
        if (index >= 0) {
            log.put("motors/${MOTOR_NAMES[index].first}/currentAmps", sampledCurrentAmps)
            sampledCurrentIndex = -1
        }
    }

    override fun writeHardware() {
        val staged = stagedManual
        stagedManual = null
        if (odometryFallback) {
            follower.drivetrain.drive(staged ?: DrivePowers.zero(), true)
            recordCommandedPower(stopped = false)
            return
        }
        if (staged != null) follower.manual(staged)
        val idle = follower.mode() == Follower.Mode.IDLE
        follower.update()
        recordCommandedPower(stopped = idle)
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

    /** Zero the motors but keep following: the next good update drives the path again. */
    override fun onWriteFault() {
        follower.drivetrain.stop()
        recordCommandedPower(stopped = true)
    }

    override fun stop() = halt()

    /** Stop following and write zero power now, without waiting for the next update. */
    private fun halt() {
        stagedManual = null
        latchedPathProgress = 0.0
        follower.stop()
        follower.drivetrain.stop()
        recordCommandedPower(stopped = true)
    }

    private companion object {
        /** Pedro's mixer order, which the `motors/power` and `motors/velocityTps` arrays follow. */
        val MOTOR_NAMES = listOf(
            "leftFront" to RobotConfig.Drive.FRONT_LEFT_MOTOR,
            "rightFront" to RobotConfig.Drive.FRONT_RIGHT_MOTOR,
            "leftRear" to RobotConfig.Drive.BACK_LEFT_MOTOR,
            "rightRear" to RobotConfig.Drive.BACK_RIGHT_MOTOR,
        )
        const val CURRENT_SAMPLE_INTERVAL_NS = 200_000_000L
        /** How long a hold or turn waits for arrival before the routine moves on. */
        const val DEFAULT_TIMEOUT_MS = 2_000.0
    }
}

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
