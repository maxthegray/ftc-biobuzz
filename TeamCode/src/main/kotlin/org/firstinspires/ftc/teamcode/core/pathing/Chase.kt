package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

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
 * Ivy scheduler will arbitrate against teleop / path-following / hold-pose
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
 */
fun chaseTarget(
    drive: MecanumDriveSubsystem,
    target: () -> Pose?,
    done: (currentTarget: Pose?) -> Boolean = { false },
): Command {
    var lastTarget: Pose? = null

    return Command.build()
        .requiring(drive)
        .setStart { lastTarget = null }
        .setExecute {
            val t = target()
            if (t != null) lastTarget = t
            lastTarget?.let { drive.holdPose(it) }
        }
        .setDone { done(lastTarget) }
}
