package org.firstinspires.ftc.teamcode.core.subsystems.drive

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.PersistedPose
import org.firstinspires.ftc.teamcode.core.runtime.SubsystemBase
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
 * Every commanding method is safe to call from inside an Ivy command.
 * Conflicting callers should declare `requiring(driveSubsystem)` on their
 * command so the scheduler arbitrates.
 */
class MecanumDriveSubsystem(val follower: Follower) : SubsystemBase("Drive") {

    enum class Mode { IDLE, TELEOP, FOLLOWING, HOLDING }

    data class TeleopInput(
        val forward: Double,
        val strafe: Double,
        val turn: Double,
        val precision: Boolean = false,
    )

    var mode: Mode = Mode.IDLE
        private set

    private var modeAfterFollow: Mode = Mode.IDLE

    override fun init(hardwareMap: HardwareMap) {
        // Follower is constructed in configure() from pedroPathing/Constants.
        // Nothing to resolve here.
    }

    /** Switch the follower into teleop drive mode. Called by [teleopCommand]. */
    internal fun enableTeleop(brakeMode: Boolean = DriveConfig.brakeOnTeleop) {
        follower.startTeleOpDrive(brakeMode)
        modeAfterFollow = Mode.IDLE
        mode = Mode.TELEOP
    }

    /**
     * Default teleop command. It starts Pedro's teleop drive mode whenever the
     * drive requirement becomes free, then applies [DriveConfig] scaling and
     * field-centric selection every tick.
     */
    fun teleopCommand(input: () -> TeleopInput): Command = Command.build()
        .requiring(this)
        .setPriority(CommandPriorities.DEFAULT)
        .setStart { enableTeleop() }
        .setExecute {
            val i = input()
            applyTeleopDrive(i.forward, i.strafe, i.turn, i.precision)
        }
        .setDone { false }

    /**
     * Imperative teleop helper retained for tests and bring-up. Prefer
     * [teleopCommand] so Ivy can automatically resume teleop after actions.
     */
    internal fun drive(forward: Double, strafe: Double, turn: Double, precision: Boolean = false) {
        applyTeleopDrive(forward, strafe, turn, precision)
    }

    private fun applyTeleopDrive(forward: Double, strafe: Double, turn: Double, precision: Boolean) {
        val scale = DriveConfig.safeTeleopPowerScale *
            (if (precision) DriveConfig.safePrecisionPowerScale else 1.0)
        val exp = DriveConfig.safeInputExponent
        val fwd = forward.curve(exp) * scale
        // FTC sticks use +x right/CW turn; Pedro uses +lateral left/CCW-positive heading.
        val strafeScaled = -strafe.curve(exp) * scale
        val turnScaled = -turn.curve(exp) * scale
        if (DriveConfig.fieldCentric) {
            follower.setTeleOpDrive(fwd, strafeScaled, turnScaled, false)
        } else {
            follower.setTeleOpDrive(fwd, strafeScaled, turnScaled, true)
        }
    }

    /**
     * Raw, unscaled drive command for tuning and drivetrain bring-up only.
     *
     * Normal op-modes should use [teleopCommand], which applies driver-feel
     * scaling and participates in Ivy's requirement arbitration.
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
        follower.followPath(chain, holdEnd)
        modeAfterFollow = if (holdEnd) Mode.HOLDING else Mode.IDLE
        mode = Mode.FOLLOWING
    }

    /** Start following a path chain with a custom max-power cap. */
    internal fun followPath(chain: PathChain, maxPower: Double, holdEnd: Boolean = true) {
        follower.followPath(chain, maxPower, holdEnd)
        modeAfterFollow = if (holdEnd) Mode.HOLDING else Mode.IDLE
        mode = Mode.FOLLOWING
    }

    /** Cancel whatever path is running and drift to idle. */
    internal fun breakPath() {
        follower.breakFollowing()
        modeAfterFollow = Mode.IDLE
        mode = Mode.IDLE
    }

    /** Pin the follower to hold a field pose, typically called at the end of an auton leg. */
    internal fun holdPose(pose: Pose) {
        follower.holdPoint(pose)
        modeAfterFollow = Mode.HOLDING
        mode = Mode.HOLDING
    }

    fun followCommand(chain: PathChain, holdEnd: Boolean = false): Command =
        trackDriveMode(
            PedroCommands.follow(follower, chain, holdEnd).asDriveAction(),
            running = Mode.FOLLOWING,
            finished = if (holdEnd) Mode.HOLDING else Mode.IDLE,
        )

    fun followCommand(chain: PathChain, maxPower: Double, holdEnd: Boolean = false): Command =
        trackDriveMode(
            PedroCommands.follow(follower, chain, holdEnd, maxPower).asDriveAction(),
            running = Mode.FOLLOWING,
            finished = if (holdEnd) Mode.HOLDING else Mode.IDLE,
        )

    /**
     * Command that holds [pose] — position *and* heading. Deliberately not
     * built on Ivy's `PedroCommands.hold`, which freezes the heading the
     * robot happened to have when the command started and ignores
     * `pose.heading`; this uses [Follower.holdPoint] directly so both
     * [holdPose] and this command agree. Done once the follower is within
     * its configured path constraints, matching Ivy's semantics.
     */
    fun holdCommand(pose: Pose): Command =
        trackDriveMode(
            Command.build()
                .setStart { follower.holdPoint(pose) }
                .setDone {
                    follower.translationalError.magnitude < follower.constraints.translationalConstraint &&
                        abs(follower.headingError) < follower.constraints.headingConstraint
                }
                .asDriveAction(),
            running = Mode.HOLDING,
            finished = Mode.HOLDING,
        )

    fun turnToCommand(radians: Double): Command =
        trackDriveMode(
            PedroCommands.turnTo(follower, radians).asDriveAction(),
            running = Mode.FOLLOWING,
            finished = Mode.IDLE,
        )

    val pose: Pose get() = follower.pose
    val velocity: Vector get() = follower.velocity

    /** True while Pedro is actively following a path. */
    val isFollowing: Boolean get() = follower.isBusy

    /** True if the robot is actually moving faster than [DriveConfig.stoppedVelocityThreshold]. */
    val isMoving: Boolean
        get() = follower.velocity.magnitude >= DriveConfig.safeStoppedVelocityThreshold

    /** Checks whether the robot is within the configured hold tolerance of a target pose. */
    fun atPose(target: Pose): Boolean {
        val current = pose
        return abs(target.x - current.x) < DriveConfig.safeHoldToleranceInches &&
            abs(target.y - current.y) < DriveConfig.safeHoldToleranceInches &&
            MathFunctions.getSmallestAngleDifference(target.heading, current.heading) <
                DriveConfig.safeHoldToleranceRadians
    }

    override fun periodic() {
        if (mode == Mode.FOLLOWING && !follower.isBusy) mode = modeAfterFollow
    }

    override fun writeHardware() {
        // Pedro's Follower.update() is a single mega-method that reads the
        // localizer, runs the path or teleop vectors, and writes motor powers.
        // Must run every tick, after the command scheduler has decided what
        // setTeleOpDrive / followPath / holdPoint call to issue.
        follower.update()
    }

    override fun persistState() {
        PersistedPose.record(follower.pose)
    }

    override fun health(): String = "mode=$mode"

    override fun stop() {
        zero()
        follower.breakFollowing()
        modeAfterFollow = Mode.IDLE
        mode = Mode.IDLE
    }

    internal fun trackDriveMode(command: Command, running: Mode, finished: Mode): Command =
        Command.build()
            .requiring(*(setOf(this) + command.requirements()).toTypedArray())
            .setPriority(command.priority())
            .setStart {
                modeAfterFollow = finished
                mode = running
                command.start()
            }
            .setExecute(command::execute)
            .setDone(command::done)
            .setEnd { endCondition ->
                command.end(endCondition)
                if (endCondition == EndCondition.INTERRUPTED) {
                    // Ivy's pedro commands have no end handler, so an
                    // interrupted follow would otherwise keep driving the
                    // path — update() runs unconditionally in writeHardware.
                    follower.breakFollowing()
                    mode = Mode.IDLE
                    modeAfterFollow = Mode.IDLE
                } else {
                    mode = finished
                }
            }
}

private fun CommandBuilder.asDriveAction(): CommandBuilder =
    setPriority(CommandPriorities.DRIVER_ACTION)

/** Applies a signed power curve: preserves sign, scales magnitude by x^exponent. */
private fun Double.curve(exponent: Double): Double = Math.copySign(Math.pow(Math.abs(this), exponent), this)
