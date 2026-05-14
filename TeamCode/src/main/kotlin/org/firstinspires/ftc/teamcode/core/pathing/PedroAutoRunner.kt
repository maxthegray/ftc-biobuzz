package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.paths.PathChain
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

/**
 * Builder + runner for an autonomous routine expressed as an Ivy command
 * sequence.
 *
 * Usage from an auton op-mode:
 *
 * ```kotlin
 * val runner = PedroAutoRunner(drive)
 *     .follow(preloadPath)
 *     .run { intake.score() }
 *     .wait(400)
 *     .follow(parkPath)
 *
 * override fun onStart() { runner.schedule() }
 * override fun onLoop() { if (runner.isDone) requestOpModeStop() }
 * ```
 *
 * Under the hood every entry becomes an [com.pedropathing.ivy.Command] and
 * the whole sequence is wrapped in `Groups.sequential` so the Ivy scheduler
 * owns execution — which means interrupting an auton routine mid-path is
 * just `Scheduler.cancel(runner.command)`.
 */
class PedroAutoRunner(private val drive: MecanumDriveSubsystem) {

    private val steps = mutableListOf<Command>()
    private var built: Command? = null

    /** Append a Pedro path to follow. */
    fun follow(chain: PathChain): PedroAutoRunner =
        append(drive.followCommand(chain, holdEnd = false))

    /** Append a Pedro path with a maximum power cap. */
    fun follow(chain: PathChain, maxPower: Double): PedroAutoRunner =
        append(drive.followCommand(chain, maxPower, holdEnd = false))

    /** Append a Pedro path and hold the end pose once it completes. */
    fun followAndHold(chain: PathChain): PedroAutoRunner =
        append(drive.followCommand(chain, holdEnd = true))

    /** Hold a fixed field pose (e.g. brake in place while mechanisms run). */
    fun holdPose(pose: Pose): PedroAutoRunner =
        append(drive.holdCommand(pose))

    /** Turn in place to an absolute heading in radians. */
    fun turnTo(radians: Double): PedroAutoRunner =
        append(drive.turnToCommand(radians))

    /**
     * Chase a moving target by feeding Pedro's holdPoint each tick. Ends
     * when [done] returns true; compose inside a `race { }` for a timeout.
     * See [chaseTarget] for details.
     */
    fun chase(
        target: () -> Pose?,
        done: (currentTarget: Pose?) -> Boolean = { false },
    ): PedroAutoRunner =
        append(chaseTarget(drive, target, done))

    /** Inject an arbitrary one-shot action (e.g. "drop pre-load"). */
    fun run(action: Runnable): PedroAutoRunner =
        append(Commands.instant(action))

    /** Wait [ms] milliseconds. */
    fun wait(ms: Double): PedroAutoRunner =
        append(Commands.waitMs(ms))

    /** Wait [ms] milliseconds (long overload, converted). */
    fun wait(ms: Long): PedroAutoRunner = wait(ms.toDouble())

    /** Inline an existing Ivy command (raw escape hatch). */
    fun then(command: Command): PedroAutoRunner =
        append(command)

    /**
     * Group sub-steps that should run in parallel. All commands inside the
     * block start together and the group completes when every one has
     * finished ("drive to stack *and* start the intake").
     */
    fun parallel(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(drive).apply(block)
        require(sub.steps.isNotEmpty()) { "parallel { } block is empty" }
        return append(Groups.parallel(*sub.steps.toTypedArray()))
    }

    /**
     * Group sub-steps into a race — they all start together, the whole
     * group finishes the instant any one of them completes. Great for
     * "drive for up to 2 s, or until the distance sensor detects the wall".
     */
    fun race(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(drive).apply(block)
        require(sub.steps.isNotEmpty()) { "race { } block is empty" }
        return append(Groups.race(*sub.steps.toTypedArray()))
    }

    private fun append(step: Command): PedroAutoRunner {
        requireMutable()
        steps += step
        return this
    }

    private fun requireMutable() {
        check(built == null) {
            "PedroAutoRunner command has already been built; add all steps before accessing command or scheduling."
        }
    }

    private fun buildInternal(): Command {
        require(steps.isNotEmpty()) { "PedroAutoRunner has no steps" }
        val arr: Array<Command> = steps.toTypedArray()
        return Groups.sequential(*arr)
    }

    /** The fully-built auton command. Rebuilt lazily on first access. */
    val command: Command
        get() {
            var b = built
            if (b == null) {
                b = buildInternal()
                built = b
            }
            return b
        }

    /** Schedule the routine on Ivy's global scheduler. */
    fun schedule(): PedroAutoRunner {
        Scheduler.schedule(command)
        return this
    }

    /** True once the routine has either completed or been cancelled. */
    val isDone: Boolean
        get() = built != null && !Scheduler.isScheduled(command)

    /** Cancel the routine mid-flight. Safe to call even if nothing is scheduled. */
    fun cancel() {
        built?.let { Scheduler.cancel(it) }
    }
}

/**
 * Convenience constructor used from op-modes to build a runner with a
 * trailing DSL block:
 *
 * ```kotlin
 * val runner = autoRoutine(drive) {
 *     follow(toPreload)
 *     run { intake.score() }
 *     wait(300)
 *     follow(park)
 * }
 * ```
 */
inline fun autoRoutine(
    drive: MecanumDriveSubsystem,
    block: PedroAutoRunner.() -> Unit,
): PedroAutoRunner = PedroAutoRunner(drive).apply(block)
