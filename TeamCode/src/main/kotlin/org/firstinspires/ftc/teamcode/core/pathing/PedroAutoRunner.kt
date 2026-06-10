package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import java.util.function.BooleanSupplier
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
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
 * just `runner.cancel()`.
 */
class PedroAutoRunner(private val drive: MecanumDriveSubsystem) {

    private val steps = mutableListOf<Command>()
    private var built: Command? = null
    private var scheduled = false
    private var timeoutMs: Double? = null

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
        reissueEpsilonInches: Double = 0.5,
        reissueHeadingEpsilonRadians: Double = Math.toRadians(5.0),
        onEnd: (EndCondition) -> Unit = { drive.breakPath() },
    ): PedroAutoRunner =
        append(
            chaseTarget(
                drive,
                target,
                done,
                reissueEpsilonInches,
                reissueHeadingEpsilonRadians,
                onEnd,
            ),
        )

    /** Inject an arbitrary one-shot action (e.g. "drop pre-load"). */
    fun run(action: Runnable): PedroAutoRunner =
        append(Commands.instant(action))

    /** Wait [ms] milliseconds. */
    fun wait(ms: Double): PedroAutoRunner =
        append(Commands.waitMs(ms))

    /** Wait [ms] milliseconds (long overload, converted). */
    fun wait(ms: Long): PedroAutoRunner = wait(ms.toDouble())

    /**
     * Wait until [condition] returns true. Has no timeout of its own — compose
     * it inside a [race] block if the routine needs to give up after a while.
     */
    fun waitUntil(condition: BooleanSupplier): PedroAutoRunner =
        append(Commands.waitUntil(condition))

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
        requireNoSharedRequirements(sub.steps, "parallel")
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
        requireNoSharedRequirements(sub.steps, "race")
        return append(Groups.race(*sub.steps.toTypedArray()))
    }

    /**
     * Group sub-steps under a deadline. The first child is the deadline; when
     * it completes, all remaining children are interrupted.
     */
    fun deadline(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(drive).apply(block)
        require(sub.steps.isNotEmpty()) { "deadline { } block is empty" }
        requireNoSharedRequirements(sub.steps, "deadline")
        val deadline = sub.steps.first()
        val others = sub.steps.drop(1).toTypedArray()
        return append(Groups.deadline(deadline, *others))
    }

    /** Race the entire built sequence against [ms] milliseconds. */
    fun timeout(ms: Double): PedroAutoRunner {
        requireMutable()
        require(ms.isFinite() && ms >= 0.0) { "timeout must be finite and non-negative" }
        timeoutMs = ms
        return this
    }

    /** Race the entire built sequence against [ms] milliseconds. */
    fun timeout(ms: Long): PedroAutoRunner = timeout(ms.toDouble())

    private fun append(step: Command): PedroAutoRunner {
        requireMutable()
        steps += step
        return this
    }

    private fun requireMutable() {
        check(built == null) {
            "PedroAutoRunner command has already been built; add all steps before build() or scheduling."
        }
    }

    private fun requireNoSharedRequirements(children: List<Command>, groupName: String) {
        val seen = HashSet<Any>()
        for (child in children) {
            for (requirement in child.requirements()) {
                require(seen.add(requirement)) {
                    "$groupName children must not share a requirement (e.g. two drive commands in one group)"
                }
            }
        }
    }

    private fun buildInternal(): Command {
        require(steps.isNotEmpty()) { "PedroAutoRunner has no steps" }
        val sequence = Groups.sequential(*steps.toTypedArray())
            .withAtLeastPriority(CommandPriorities.AUTON_ROUTINE)
        val timeout = timeoutMs
        return if (timeout == null) {
            sequence
        } else {
            Groups.race(sequence, Commands.waitMs(timeout))
                .withAtLeastPriority(CommandPriorities.AUTON_ROUTINE)
        }
    }

    /** Build and lock the routine into one Ivy command. Further steps cannot be appended. */
    fun build(): Command {
        var b = built
        if (b == null) {
            b = buildInternal()
            built = b
        }
        return b
    }

    /** Schedule the routine on Ivy's global scheduler. */
    fun schedule(): PedroAutoRunner {
        if (scheduled) return this
        Scheduler.schedule(build())
        scheduled = true
        return this
    }

    /** True once the routine has either completed or been cancelled. */
    val isDone: Boolean
        get() = scheduled && built?.let { !Scheduler.isScheduled(it) } == true

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

private fun CommandBuilder.withAtLeastPriority(priority: Int): CommandBuilder =
    setPriority(kotlin.math.max(priority(), priority))
