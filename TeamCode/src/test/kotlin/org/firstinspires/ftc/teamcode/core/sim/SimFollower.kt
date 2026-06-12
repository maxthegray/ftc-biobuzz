package org.firstinspires.ftc.teamcode.core.sim

import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.geometry.Pose
import com.pedropathing.localization.Localizer
import com.pedropathing.drivetrain.Drivetrain
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.withSign
import org.firstinspires.ftc.teamcode.core.util.FakeClock

/**
 * Headless, kinematic "perfect follower" for JVM tests.
 *
 * It honours the same public surface the framework actually calls —
 * `followPath(chain/path, …)` + `isBusy()` (used by the drive subsystem's
 * follow / turnTo commands), `holdPoint(...)` +
 * `getTranslationalError()` / `getHeadingError()` (used by
 * `MecanumDriveSubsystem.holdCommand`), teleop drive vectors, and the pose
 * accessors — but replaces Pedro's control law with straight-line motion at
 * [linearSpeedInPerSec] / [angularSpeedRadPerSec], advanced by [update] using
 * the shared [FakeClock]. Path *geometry* stays real: targets are read from
 * the actual [PathChain] built by `PathDSL`, so alliance mirroring and
 * heading interpolation are exercised end to end.
 *
 * This validates routine logic (sequencing, mirroring, mode transitions,
 * pose handoff), not Pedro's control quality — tune that on carpet.
 */
class SimFollower(
    private val clock: FakeClock,
    var linearSpeedInPerSec: Double = 40.0,
    var angularSpeedRadPerSec: Double = Math.toRadians(270.0),
) : Follower(FollowerConstants(), SimLocalizer(), SimDrivetrain()) {

    private var poseState = Pose()
    private var velocityState = Vector()
    private var lastUpdateNs = Long.MIN_VALUE

    private val legs = ArrayDeque<Pose>()
    private var holdEndAfterLegs = false
    private var holding: Pose? = null
    private var busyState = false
    private var activeChain: PathChain? = null
    private var totalLegs = 0
    private var completedLegs = 0
    private var currentLegStartDistance = -1.0
    private var teleop = false
    private var teleopForward = 0.0
    private var teleopStrafe = 0.0
    private var teleopTurn = 0.0
    private var teleopRobotCentric = true

    var breakFollowingCalls = 0
        private set

    // ------------------------------------------------------------------ pose

    override fun getPose(): Pose = poseState

    override fun setPose(pose: Pose) {
        poseState = pose
    }

    override fun setStartingPose(pose: Pose) {
        poseState = pose
    }

    override fun getVelocity(): Vector = velocityState

    // -------------------------------------------------------------- following

    override fun followPath(pathChain: PathChain, maxPower: Double, holdEnd: Boolean) {
        clearMotion()
        for (i in 0 until pathChain.size()) {
            val path = pathChain.getPath(i)
            legs += targetOf(path)
        }
        holdEndAfterLegs = holdEnd
        busyState = legs.isNotEmpty()
        activeChain = pathChain
        totalLegs = legs.size
    }

    override fun followPath(pathChain: PathChain, holdEnd: Boolean) =
        followPath(pathChain, 1.0, holdEnd)

    override fun followPath(pathChain: PathChain) = followPath(pathChain, 1.0, false)

    override fun followPath(path: Path, holdEnd: Boolean) {
        clearMotion()
        legs += targetOf(path)
        holdEndAfterLegs = holdEnd
        busyState = true
        totalLegs = 1
    }

    override fun followPath(path: Path) = followPath(path, false)

    override fun holdPoint(point: BezierPoint, heading: Double, useHoldScaling: Boolean) {
        val p = point.lastControlPoint
        clearMotion()
        holding = Pose(p.x, p.y, MathFunctions.normalizeAngle(heading))
    }

    override fun holdPoint(point: BezierPoint, heading: Double) =
        holdPoint(point, heading, true)

    override fun holdPoint(pose: Pose) = holdPoint(BezierPoint(pose), pose.heading, true)

    override fun breakFollowing() {
        // The Follower constructor calls breakFollowing() before this class's
        // field initializers have run — ignore that call.
        @Suppress("SENSELESS_COMPARISON")
        if (legs == null) return
        breakFollowingCalls++
        clearMotion()
    }

    override fun isBusy(): Boolean = busyState

    // ------------------------------------------------------------- progress
    // The exact base-Follower surface MecanumDriveSubsystem.pathProgress()
    // reads, emulated so progress markers are sim-testable. The per-leg
    // parameter approximates Pedro's t-value with travelled-distance
    // fraction — fine for marker logic, not for control.

    override fun getCurrentPathChain(): PathChain? = activeChain

    override fun getCurrentPathNumber(): Double =
        completedLegs.coerceAtMost((totalLegs - 1).coerceAtLeast(0)).toDouble()

    override fun getCurrentTValue(): Double {
        if (totalLegs == 0) return 0.0
        if (legs.isEmpty()) return 1.0
        val start = currentLegStartDistance
        if (start <= 1e-9) return 0.0
        val target = legs.first()
        val remaining = hypot(target.x - poseState.x, target.y - poseState.y)
        return (1.0 - remaining / start).coerceIn(0.0, 1.0)
    }

    // ----------------------------------------------------------------- teleop

    override fun startTeleopDrive(useBrakeMode: Boolean) {
        clearMotion()
        teleop = true
    }

    override fun startTeleOpDrive(useBrakeMode: Boolean) = startTeleopDrive(useBrakeMode)

    override fun setTeleOpDrive(forward: Double, strafe: Double, turn: Double, isRobotCentric: Boolean) {
        teleopForward = forward
        teleopStrafe = strafe
        teleopTurn = turn
        teleopRobotCentric = isRobotCentric
    }

    // ----------------------------------------------------------------- errors

    override fun getTranslationalError(): Vector {
        val target = holding ?: legs.firstOrNull() ?: return Vector()
        val dx = target.x - poseState.x
        val dy = target.y - poseState.y
        return Vector(hypot(dx, dy), atan2(dy, dx))
    }

    override fun getHeadingError(): Double {
        val target = holding ?: legs.firstOrNull() ?: return 0.0
        return MathFunctions.normalizeAngleSigned(target.heading - poseState.heading)
    }

    // ------------------------------------------------------------------- tick

    override fun update() {
        val now = clock.nanos()
        val dtSeconds =
            if (lastUpdateNs == Long.MIN_VALUE) 0.0 else (now - lastUpdateNs) / 1e9
        lastUpdateNs = now
        if (dtSeconds <= 0.0) {
            velocityState = Vector()
            return
        }

        val before = poseState
        when {
            busyState && legs.isNotEmpty() -> {
                if (currentLegStartDistance < 0.0) {
                    val target = legs.first()
                    currentLegStartDistance = hypot(target.x - poseState.x, target.y - poseState.y)
                }
                moveToward(legs.first(), dtSeconds)
                if (arrived(legs.first())) {
                    val finished = legs.removeFirst()
                    completedLegs++
                    currentLegStartDistance = -1.0
                    if (legs.isEmpty()) {
                        busyState = false
                        if (holdEndAfterLegs) holding = finished
                    }
                }
            }
            holding != null -> moveToward(holding!!, dtSeconds)
            teleop -> integrateTeleop(dtSeconds)
        }

        val travelled = hypot(poseState.x - before.x, poseState.y - before.y)
        velocityState =
            if (travelled > 0.0) {
                Vector(
                    travelled / dtSeconds,
                    atan2(poseState.y - before.y, poseState.x - before.x),
                )
            } else {
                Vector()
            }
    }

    // -------------------------------------------------------------- internals

    private fun targetOf(path: Path): Pose {
        val end = path.lastControlPoint
        val heading = try {
            path.getHeadingGoal(1.0)
        } catch (_: RuntimeException) {
            poseState.heading
        }
        return Pose(end.x, end.y, MathFunctions.normalizeAngle(heading))
    }

    private fun moveToward(target: Pose, dtSeconds: Double) {
        val dx = target.x - poseState.x
        val dy = target.y - poseState.y
        val distance = hypot(dx, dy)
        val step = min(linearSpeedInPerSec * dtSeconds, distance)
        val (nx, ny) =
            if (distance > 1e-9) {
                Pair(poseState.x + dx / distance * step, poseState.y + dy / distance * step)
            } else {
                Pair(target.x, target.y)
            }

        val headingDelta = MathFunctions.normalizeAngleSigned(target.heading - poseState.heading)
        val headingStep = min(angularSpeedRadPerSec * dtSeconds, abs(headingDelta))
        val nh = MathFunctions.normalizeAngle(poseState.heading + headingStep.withSign(headingDelta))

        poseState = Pose(nx, ny, nh)
    }

    private fun arrived(target: Pose): Boolean =
        hypot(target.x - poseState.x, target.y - poseState.y) < 1e-6 &&
            abs(MathFunctions.normalizeAngleSigned(target.heading - poseState.heading)) < 1e-6

    private fun integrateTeleop(dtSeconds: Double) {
        val vx: Double
        val vy: Double
        if (teleopRobotCentric) {
            val cos = Math.cos(poseState.heading)
            val sin = Math.sin(poseState.heading)
            vx = (teleopForward * cos - teleopStrafe * sin) * linearSpeedInPerSec
            vy = (teleopForward * sin + teleopStrafe * cos) * linearSpeedInPerSec
        } else {
            vx = teleopForward * linearSpeedInPerSec
            vy = teleopStrafe * linearSpeedInPerSec
        }
        poseState = Pose(
            poseState.x + vx * dtSeconds,
            poseState.y + vy * dtSeconds,
            MathFunctions.normalizeAngle(
                poseState.heading + teleopTurn * angularSpeedRadPerSec * dtSeconds,
            ),
        )
    }

    private fun clearMotion() {
        legs.clear()
        holdEndAfterLegs = false
        holding = null
        busyState = false
        teleop = false
        teleopForward = 0.0
        teleopStrafe = 0.0
        teleopTurn = 0.0
        activeChain = null
        totalLegs = 0
        completedLegs = 0
        currentLegStartDistance = -1.0
    }
}

private class SimLocalizer : Localizer {
    private var pose = Pose()

    override fun getPose(): Pose = pose
    override fun getVelocity(): Pose = Pose()
    override fun getVelocityVector(): Vector = Vector()
    override fun setStartPose(setStart: Pose) { pose = setStart }
    override fun setPose(setPose: Pose) { pose = setPose }
    override fun update() {}
    override fun getTotalHeading(): Double = pose.heading
    override fun getForwardMultiplier(): Double = 1.0
    override fun getLateralMultiplier(): Double = 1.0
    override fun getTurningMultiplier(): Double = 1.0
    override fun resetIMU() {}
    override fun getIMUHeading(): Double = pose.heading
    override fun isNAN(): Boolean = false
}

private class SimDrivetrain : Drivetrain() {
    override fun calculateDrive(
        correctivePower: Vector,
        headingPower: Vector,
        drivePower: Vector,
        robotHeading: Double,
    ): DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    override fun updateConstants() {}
    override fun breakFollowing() {}
    override fun runDrive(powers: DoubleArray) {}
    override fun startTeleopDrive() {}
    override fun startTeleopDrive(brake: Boolean) {}
    override fun xVelocity(): Double = 0.0
    override fun yVelocity(): Double = 0.0
    override fun setXVelocity(xVelocity: Double) {}
    override fun setYVelocity(yVelocity: Double) {}
    override fun getVoltage(): Double = 12.0
    override fun debugString(): String = "sim"
}
