package org.firstinspires.ftc.teamcode.starter.drive

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.starter.core.SubsystemBase
import kotlin.math.hypot

/**
 * The one subsystem that owns the mecanum drivetrain. It is a thin façade
 * over the Pedro [Follower] — Pedro already handles motor wiring, field-
 * centric projection, the P/I/D/F loops, and path following. This class
 * exists to:
 *
 *  - Slot into [org.firstinspires.ftc.teamcode.starter.core.Robot]'s
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

    var mode: Mode = Mode.IDLE
        private set

    override fun init(hardwareMap: HardwareMap) {
        // Follower is constructed in configure() from pedroPathing/Constants.
        // Nothing to resolve here.
    }

    /** Switch the follower into teleop drive mode. Call once at teleop start. */
    fun enableTeleop(brakeMode: Boolean = DriveConfig.brakeOnTeleop) {
        follower.startTeleOpDrive(brakeMode)
        mode = Mode.TELEOP
    }

    /**
     * Primary teleop entry point. Applies DriveConfig scaling and the
     * current field-centric setting, then hands off to the follower.
     *
     * Inputs are in the idiomatic FTC stick convention: forward is
     * `-leftStickY`, strafe is `leftStickX`, turn is `rightStickX`, each
     * in `[-1.0, 1.0]`.
     */
    fun drive(forward: Double, strafe: Double, turn: Double, precision: Boolean = false) {
        val scale = DriveConfig.teleopPowerScale * (if (precision) DriveConfig.precisionPowerScale else 1.0)
        val fwd = forward * scale
        val strafeScaled = strafe * scale
        val turnScaled = turn * scale
        if (DriveConfig.fieldCentric) {
            follower.setTeleOpDrive(fwd, strafeScaled, turnScaled, false)
        } else {
            follower.setTeleOpDrive(fwd, strafeScaled, turnScaled, true)
        }
    }

    /** Raw, unscaled drive command. Useful for PID tuning or fallback control. */
    fun driveRaw(forward: Double, strafe: Double, turn: Double, robotCentric: Boolean) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric)
    }

    /** Zero the teleop movement vectors — robot coasts/brakes per [DriveConfig.brakeOnTeleop]. */
    fun zero() {
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
    }

    /** Start following a pre-built path chain. */
    fun followPath(chain: PathChain, holdEnd: Boolean = true) {
        follower.followPath(chain, holdEnd)
        mode = Mode.FOLLOWING
    }

    /** Start following a path chain with a custom max-power cap. */
    fun followPath(chain: PathChain, maxPower: Double, holdEnd: Boolean = true) {
        follower.followPath(chain, maxPower, holdEnd)
        mode = Mode.FOLLOWING
    }

    /** Cancel whatever path is running and drift to idle. */
    fun breakPath() {
        follower.breakFollowing()
        mode = Mode.IDLE
    }

    /** Pin the follower to hold a field pose, typically called at the end of an auton leg. */
    fun holdPose(pose: Pose) {
        follower.holdPoint(pose)
        mode = Mode.HOLDING
    }

    val pose: Pose get() = follower.pose
    val velocity: Pose get() = follower.velocity

    /** True while Pedro is actively following a path. */
    val isFollowing: Boolean get() = follower.isBusy

    /** True if the robot is actually moving faster than [DriveConfig.stoppedVelocityThreshold]. */
    val isMoving: Boolean
        get() {
            val v = follower.velocity
            return hypot(v.x, v.y) >= DriveConfig.stoppedVelocityThreshold
        }

    /** Checks whether the robot is within the configured hold tolerance of a target pose. */
    fun atPose(target: Pose): Boolean =
        follower.atPose(
            target,
            DriveConfig.holdToleranceInches,
            DriveConfig.holdToleranceInches,
            DriveConfig.holdToleranceRadians,
        )

    override fun writeHardware() {
        // Pedro's Follower.update() is a single mega-method that reads the
        // localizer, runs the path or teleop vectors, and writes motor powers.
        // Must run every tick, after the command scheduler has decided what
        // setTeleOpDrive / followPath / holdPoint call to issue.
        follower.update()
    }

    override fun stop() {
        zero()
        follower.breakFollowing()
    }
}
