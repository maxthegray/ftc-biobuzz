package org.firstinspires.ftc.teamcode.core.pathing

import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.CommandBuilder
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.command.Groups
import org.firstinspires.ftc.teamcode.core.geometry.Pose2d
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem

/**
 * Builder + runner for an autonomous routine expressed as a command sequence.
 *
 * Usage from an auton op-mode:
 *
 * ```kotlin
 * val runner = PedroAutoRunner(robot, drive)
 *     .follow(preloadPath)
 *     .run { intake.score() }
 *     .wait(400)
 *     .follow(parkPath)
 *
 * override fun onStart() { runner.schedule() }
 * override fun onLoop() { if (runner.isDone) requestOpModeStop() }
 * ```
 *
 * Under the hood every entry becomes a [Command] and the whole sequence is
 * wrapped in `Groups.sequential`, scheduled on the robot's scheduler — which
 * means interrupting an auton routine mid-path is just `runner.cancel()`.
 *
 * Waits and [timeout]s run on the robot's [org.firstinspires.ftc.teamcode.core.util.Clock],
 * so routines — including their waits — execute under virtual time in the
 * headless sim.
 *
 * Pass an [onEvent] sink (typically `robot::recordEvent`) and the runner
 * emits a labelled event as each step starts — every flight log then carries
 * a navigable timeline of the routine.
 */
class PedroAutoRunner(
    private val robot: Robot,
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

    /**
     * Follow [chain] with progress markers: each `at(t) { … }` action fires
     * once when [MecanumDriveSubsystem.pathProgress] crosses `t` (0..1 over
     * the whole chain). The step completes when the follow does; markers the
     * path never reached are dropped (e.g. when the routine is cancelled
     * mid-path).
     *
     * ```kotlin
     * follow(toScore) {
     *     at(0.3) { lift.setGoal(HIGH) }
     *     at(0.85, "deploy") { intake.deploy() }
     * }
     * ```
     */
    fun follow(chain: PathChain, markers: MarkerScope.() -> Unit): PedroAutoRunner =
        followWithMarkers(drive.followCommand(chain, holdEnd = false), "follow", markers)

    /** [follow]-with-markers, holding the end pose once the path completes. */
    fun followAndHold(chain: PathChain, markers: MarkerScope.() -> Unit): PedroAutoRunner =
        followWithMarkers(drive.followCommand(chain, holdEnd = true), "followAndHold", markers)

    /** Receiver for the marker blocks of [follow] / [followAndHold]. */
    inner class MarkerScope internal constructor() {
        internal val markers = mutableListOf<Command>()

        /** Run [action] once, when chain progress first reaches [progress]. */
        fun at(progress: Double, label: String = "marker", action: () -> Unit) {
            require(progress in 0.0..1.0) { "marker progress must be in 0..1, got $progress" }
            val fire = {
                onEvent?.invoke("$label @ %.0f%%".format(java.util.Locale.US, progress * 100))
                action()
            }
            markers += Groups.sequential(
                Commands.waitUntil { drive.pathProgress() >= progress }
                    .setName("until $progress"),
                Commands.instant(fire).setName(label),
            ).setName("$label@$progress")
        }
    }

    private fun followWithMarkers(
        followCommand: Command,
        label: String,
        block: MarkerScope.() -> Unit,
    ): PedroAutoRunner {
        requireMutable()
        val scope = MarkerScope().apply(block)
        if (scope.markers.isEmpty()) return append(label, followCommand)
        // Deadline semantics: the follow is the deadline; unfired markers are
        // interrupted the moment it completes (or the routine is cancelled).
        return append(
            "$label+markers(${scope.markers.size})",
            Groups.deadline(followCommand, *scope.markers.toTypedArray()),
        )
    }

    /** Hold a fixed field pose (e.g. brake in place while mechanisms run). */
    fun holdPose(pose: Pose2d): PedroAutoRunner =
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
        target: () -> Pose2d?,
        done: (currentTarget: Pose2d?) -> Boolean = { false },
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
        append("run", Commands.instant { action.run() })

    /** Wait [ms] milliseconds (on the robot's clock — simulable). */
    fun wait(ms: Double): PedroAutoRunner =
        append("wait ${ms.toLong()} ms", Commands.waitMs(ms, robot.clock))

    /** Wait [ms] milliseconds (long overload, converted). */
    fun wait(ms: Long): PedroAutoRunner = wait(ms.toDouble())

    /**
     * Wait until [condition] returns true. Has no timeout of its own — compose
     * it inside a [race] block if the routine needs to give up after a while.
     */
    fun waitUntil(condition: () -> Boolean): PedroAutoRunner =
        append("waitUntil", Commands.waitUntil(condition))

    /** Inline an existing command (raw escape hatch). */
    fun then(command: Command): PedroAutoRunner =
        append("command", command)

    /**
     * Group sub-steps that should run in parallel. All commands inside the
     * block start together and the group completes when every one has
     * finished ("drive to stack *and* start the intake").
     */
    fun parallel(block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        val sub = PedroAutoRunner(robot, drive).apply(block)
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
        val sub = PedroAutoRunner(robot, drive).apply(block)
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
        val sub = PedroAutoRunner(robot, drive).apply(block)
        require(sub.steps.isNotEmpty()) { "deadline { } block is empty" }
        requireNoSharedRequirements(sub.commands(), "deadline")
        val deadline = sub.commands().first()
        val others = sub.commands().drop(1).toTypedArray()
        return append(sub.groupLabel("deadline"), Groups.deadline(deadline, *others))
    }

    /** Race the entire built sequence against [ms] milliseconds (robot clock). */
    fun timeout(ms: Double): PedroAutoRunner {
        requireMutable()
        require(ms.isFinite() && ms >= 0.0) { "timeout must be finite and non-negative" }
        timeoutMs = ms
        return this
    }

    /** Race the entire built sequence against [ms] milliseconds (robot clock). */
    fun timeout(ms: Long): PedroAutoRunner = timeout(ms.toDouble())

    /**
     * Race the sub-steps in [block] against [ms] milliseconds (robot clock):
     * if they don't finish in time they are interrupted (a follow breaks its
     * path) and the routine moves on. This is the per-step guard against a
     * stalled mechanism or unreachable pose eating the rest of auton —
     * the no-block [timeout] overload still bounds the whole routine.
     *
     * ```kotlin
     * timeout(2000.0) { follow(toScore) }
     * ```
     */
    fun timeout(ms: Double, block: PedroAutoRunner.() -> Unit): PedroAutoRunner {
        requireMutable()
        require(ms.isFinite() && ms >= 0.0) { "timeout must be finite and non-negative" }
        val sub = PedroAutoRunner(robot, drive).apply(block)
        require(sub.steps.isNotEmpty()) { "timeout { } block is empty" }
        val sequence: Command = if (sub.steps.size == 1) {
            sub.commands().first()
        } else {
            Groups.sequential(*sub.commands().toTypedArray())
        }
        return append(
            sub.groupLabel("timeout ${ms.toLong()} ms"),
            Groups.race(sequence, Commands.waitMs(ms, robot.clock)),
        )
    }

    /** [timeout]-with-steps, long overload for literal milliseconds. */
    fun timeout(ms: Long, block: PedroAutoRunner.() -> Unit): PedroAutoRunner =
        timeout(ms.toDouble(), block)

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
            .setName("auto routine")
            .withAtLeastPriority(CommandPriorities.AUTON_ROUTINE)
        val timeout = timeoutMs
        return if (timeout == null) {
            sequence
        } else {
            Groups.race(sequence, Commands.waitMs(timeout, robot.clock))
                .setName("auto routine (timeout ${timeout.toLong()} ms)")
                .withAtLeastPriority(CommandPriorities.AUTON_ROUTINE)
        }
    }

    /** Build and lock the routine into one command. Further steps cannot be appended. */
    fun build(): Command {
        var b = built
        if (b == null) {
            b = buildInternal()
            built = b
        }
        return b
    }

    /** Schedule the routine on the robot's scheduler. */
    fun schedule(): PedroAutoRunner {
        if (scheduled) return this
        robot.scheduler.schedule(build())
        scheduled = true
        return this
    }

    /** True once the routine has either completed or been cancelled. */
    val isDone: Boolean
        get() = scheduled && built?.let { !robot.scheduler.isScheduled(it) } == true

    /** Cancel the routine mid-flight. Safe to call even if nothing is scheduled. */
    fun cancel() {
        built?.let { robot.scheduler.cancel(it) }
    }
}

/**
 * Convenience constructor used from op-modes to build a runner with a
 * trailing DSL block:
 *
 * ```kotlin
 * val runner = autoRoutine(robot, drive, robot::recordEvent) {
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
    robot: Robot,
    drive: MecanumDriveSubsystem,
    noinline onEvent: ((String) -> Unit)? = null,
    block: PedroAutoRunner.() -> Unit,
): PedroAutoRunner = PedroAutoRunner(robot, drive, onEvent).apply(block)

private fun CommandBuilder.withAtLeastPriority(priority: Int): CommandBuilder =
    setPriority(kotlin.math.max(priority(), priority))
