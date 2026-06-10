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
 *
 * Pass an [onEvent] sink (typically `robot::recordEvent`) and the runner
 * emits a labelled event as each step starts — every flight log then carries
 * a navigable timeline of the routine.
 */
class PedroAutoRunner(
    private val drive: MecanumDriveSubsystem,
    private val onEvent: ((String) -> Unit)? = null,
) {

    private class Step(val label: String, val command: Command)

    private val steps = mutableListOf<Step>()
    private var built: Command? = null
    private var scheduled = false
    private var timeoutMs: Double? = null

    /** Append a Pedro path to follow. */
    fun follow(chain: PathChain): PedroAutoRunner =
        append("follow", drive.followCommand(chain, holdEnd = false))

    /** Append a Pedro path with a maximum power cap. */
    fun follow(chain: PathChain, maxPower: Double): PedroAutoRunner =
        append("follow (maxPower=$maxPower)", drive.followCommand(chain, maxPower, holdEnd = false))

    /** Append a Pedro path and hold the end pose once it completes. */
    fun followAndHold(chain: PathChain): PedroAutoRunner =
        append("followAndHold", drive.followCommand(chain, holdEnd = true))

    /** Hold a fixed field pose (e.g. brake in place while mechanisms run). */
    fun holdPose(pose: Pose): PedroAutoRunner =
        append("holdPose", drive.holdCommand(pose))

    /** Turn in place to an absolute heading in radians. */
    fun turnTo(radians: Double): PedroAutoRunner =
        append("turnTo %.0f deg".format(java.util.Locale.US, Math.toDegrees(radians)), drive.turnToCommand(radians))

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
            "chase",
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
        append("run", Commands.instant(action))

    /** Wait [ms] milliseconds. */
    fun wait(ms: Double): PedroAutoRunner =
        append("wait ${ms.toLong()} ms", Commands.waitMs(ms))

    /** Wait [ms] milliseconds (long overload, converted). */
    fun wait(ms: Long): PedroAutoRunner = wait(ms.toDouble())

    /**
     * Wait until [condition] returns true. Has no timeout of its own — compose
     * it inside a [race] block if the routine needs to give up after a while.
     */
    fun waitUntil(condition: BooleanSupplier): PedroAutoRunner =
        append("waitUntil", Commands.waitUntil(condition))

    /** Inline an existing Ivy command (raw escape hatch). */
    fun then(command: Command): PedroAutoRunner =
        append("command", command)

    /**
     * Group sub-steps that should run in parallel. All commands inside the
     * block start together and the group completes when every one has
     * finished ("drive to stack *and* start the intake").
     */
    fun parallel(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(drive).apply(block)
        require(sub.steps.isNotEmpty()) { "parallel { } block is empty" }
        requireNoSharedRequirements(sub.commands(), "parallel")
        return append(sub.groupLabel("parallel"), Groups.parallel(*sub.commands().toTypedArray()))
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
        requireNoSharedRequirements(sub.commands(), "race")
        return append(sub.groupLabel("race"), Groups.race(*sub.commands().toTypedArray()))
    }

    /**
     * Group sub-steps under a deadline. The first child is the deadline; when
     * it completes, all remaining children are interrupted.
     */
    fun deadline(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(drive).apply(block)
        require(sub.steps.isNotEmpty()) { "deadline { } block is empty" }
        requireNoSharedRequirements(sub.commands(), "deadline")
        val deadline = sub.commands().first()
        val others = sub.commands().drop(1).toTypedArray()
        return append(sub.groupLabel("deadline"), Groups.deadline(deadline, *others))
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

    private fun append(label: String, step: Command): PedroAutoRunner {
        requireMutable()
        steps += Step(label, step)
        return this
    }

    private fun commands(): List<Command> = steps.map { it.command }

    private fun groupLabel(groupName: String): String =
        steps.joinToString(prefix = "$groupName[", postfix = "]") { it.label }

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
        val events = onEvent
        val sequenced: List<Command> = if (events == null) {
            commands()
        } else {
            val total = steps.size
            buildList {
                steps.forEachIndexed { index, step ->
                    add(Commands.instant { events("auto step ${index + 1}/$total: ${step.label}") })
                    add(step.command)
                }
                add(Commands.instant { events("auto routine complete") })
            }
        }
        val sequence = Groups.sequential(*sequenced.toTypedArray())
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
 * val runner = autoRoutine(drive, robot::recordEvent) {
 *     follow(toPreload)
 *     run { intake.score() }
 *     wait(300)
 *     follow(park)
 * }
 * ```
 *
 * Pass `robot::recordEvent` as [onEvent] to get a labelled step timeline in
 * the flight log; omit it for silent sequencing.
 */
inline fun autoRoutine(
    drive: MecanumDriveSubsystem,
    noinline onEvent: ((String) -> Unit)? = null,
    block: PedroAutoRunner.() -> Unit,
): PedroAutoRunner = PedroAutoRunner(drive, onEvent).apply(block)

private fun CommandBuilder.withAtLeastPriority(priority: Int): CommandBuilder =
    setPriority(kotlin.math.max(priority(), priority))
