package org.firstinspires.ftc.teamcode.core.pathing

import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.geometry.shortestAngleDelta
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Dynamic-target chaser built on top of Pedro's `holdPoint`.
 *
 * Every tick, [target] is polled. If it returns a [Pose], the drive's
 * holdPoint setpoint is moved there; Pedro's translation + heading PIDs do
 * the actual tracking. If [target] returns `null`, the *last known* setpoint
 * is left in place — i.e. one missed frame won't halt the robot. Use [done]
 * to define when the command should end (e.g. "we've captured the ball" or
 * "target has been lost for too long").
 *
 * The command requires [drive] like any other drive-claiming command, so the
 * scheduler will arbitrate against teleop / path-following / hold-pose
 * commands.
 *
 * Caveat: `holdPoint` was designed for stationary setpoints; if the target
 * moves fast or stays far away, Pedro's integral term can wind up. If you
 * see overshoot or oscillation while chasing, lower the I gain on the
 * translation PID in your Pedro Constants — this command does not (and
 * should not) touch follower tuning itself.
 *
 * @param drive  The drive subsystem to claim.
 * @param target Polled every tick. Return `null` to indicate "no fix this
 *               frame" — the chaser will hold the last known target.
 * @param done   Optional end condition. Receives the latest non-null target
 *               (or `null` if one has never been seen) so callers can write
 *               capture checks like `{ t -> t != null && drive.atPose(t) }`.
 *               Default: never ends — compose with `race { ... }` or a
 *               timeout if you want bounded chasing.
 * @param reissueEpsilonInches Minimum setpoint move before `holdPoint` is
 *               re-issued. Re-issuing resets the hold controller and allocates,
 *               so small jitter below this threshold is ignored.
 * @param reissueHeadingEpsilonRadians Minimum heading change before `holdPoint`
 *               is re-issued, using shortest-angle wraparound.
 * @param onEnd Called whenever the command ends. The default breaks Pedro's
 *               hold mode so a timeout does not leave the robot chasing a stale
 *               setpoint.
 */
fun chaseTarget(
    drive: MecanumDriveSubsystem,
    target: () -> Pose2d?,
    done: (currentTarget: Pose2d?) -> Boolean = { false },
    reissueEpsilonInches: Double = 0.5,
    reissueHeadingEpsilonRadians: Double = Math.toRadians(5.0),
    onEnd: (EndCondition) -> Unit = { drive.breakPath() },
): Command {
    require(reissueEpsilonInches.isFinite() && reissueEpsilonInches >= 0.0) {
        "reissueEpsilonInches must be finite and non-negative"
    }
    require(reissueHeadingEpsilonRadians.isFinite() && reissueHeadingEpsilonRadians >= 0.0) {
        "reissueHeadingEpsilonRadians must be finite and non-negative"
    }

    // Two distinct pieces of state: `seenTarget` is null until a real target
    // is observed (that's what `done` is contracted to receive), while
    // `holdSetpoint` is the pose we actually hold — seeded to the start pose so
    // the robot pins in place instead of drifting before the first fix.
    var seenTarget: Pose2d? = null
    var holdSetpoint: Pose2d? = null
    var issued: Pose2d? = null

    return Command.build()
        .setName("chase")
        .requiring(drive)
        .setStart {
            seenTarget = null
            holdSetpoint = drive.pose
            issued = null
        }
        .setExecute {
            target()?.let {
                seenTarget = it
                holdSetpoint = it
            }
            val setpoint = holdSetpoint ?: return@setExecute
            // Only re-issue holdPoint when the setpoint actually moved: each call
            // breaks following, allocates a fresh BezierPoint + Path, and resets
            // the hold controller, so spamming it every tick is wasteful.
            val prev = issued
            val positionMoved = prev == null ||
                hypot(setpoint.x - prev.x, setpoint.y - prev.y) >= reissueEpsilonInches
            val headingMoved = prev == null ||
                abs(shortestAngleDelta(prev.heading, setpoint.heading)) >=
                    reissueHeadingEpsilonRadians
            if (positionMoved || headingMoved) {
                drive.holdPose(setpoint)
                issued = setpoint
            }
        }
        .setDone { done(seenTarget) }
        .setEnd(onEnd)
}
