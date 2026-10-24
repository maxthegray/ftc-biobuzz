package org.firstinspires.ftc.teamcode.opmodes.skeletons

import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Minimal end-to-end auton: drive out 24", settle, "score" (a wait), turn,
 * and drive back. Not game code; copy it as the skeleton for a real routine.
 *
 *  - Poses are written once in RED coordinates through [Alliance.poses]
 *    (degrees); BLUE gets them mirrored. Bare headings go through
 *    [Alliance.mirror].
 *  - Paths come from Pedro's `Paths` API; `.linear(a, b)` turns along a path.
 *  - The routine is plain Ivy composition: `sequential`, `race` for timeouts,
 *    `deadline` for a mid-path marker that fires once and is dropped if the
 *    path ends first. Waits and timeouts use Ivy's `waitMs`.
 *  - The routine and its meaningful steps are [logged], so `commands/events`
 *    shows which step ran, finished, was interrupted or failed.
 *  - One op-mode per alliance and routine: copy this file and override
 *    [initialAlliance] for BLUE. Only the start delay is picked at init.
 *  - Lifecycle: the start pose is written to the Pinpoint at INIT (place the
 *    robot first); refuse to start without Foresight tuning; wait up to
 *    [LOCALIZER_WAIT_MS] for a ready localizer, then run anyway; only a lost
 *    pose cancels the routine, Pinpoint status problems are just logged;
 *    require scheduling to succeed; stop
 *    when the routine is no longer scheduled. Teleop starts again at (0, 0, 0).
 *
 * Disabled: it needs Foresight tuning and real poses before it means anything.
 */
@Disabled
@Autonomous(name = "Example Auto", group = "Match")
class ExampleAuto : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    // Picked on dpad left/right during init, agreed with the alliance partner before the match.
    private var startDelaySec = 0
    private var routine: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L

    /** The BLUE copy of this file overrides this and changes nothing else. */
    override val initialAlliance: Alliance get() = Alliance.RED

    private val poses get() = alliance.poses()
    private val start get() = poses.of(8.0, 56.0, 0.0)
    private val out get() = poses.of(32.0, 56.0, 0.0)
    private val outTurned get() = poses.of(32.0, 56.0, 90.0)

    override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        // Drive first, localizer second: pose history is sampled after Follower.update().
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                onEvent = robot::recordEvent,
                isFollowing = drive::isFollowing,
                // Driving blind is worse than parking: cancel the routine (its
                // drive command stops the follower) and stay put.
                onFault = { routine?.let(Scheduler::cancel) },
                // The pose is still live: keep driving, the event is in the log.
                onStatusFault = {},
                // Written to the Pinpoint at INIT, after the reset. Place the robot before INIT.
                startingPose = start,
            ),
        )
    }

    private fun outPath(): Path = line(start, out).constant(start)

    private fun backPath(): Path = line(outTurned, start).linear(outTurned, start)

    // Named steps show up in commands/events and commands/active. The drive
    // factories are already logged (pass `name`); other steps are wrapped with
    // logged(). The marker and the instants stay unlogged: they are events.
    private fun buildRoutine(): Command = logged(
        "Example Auto routine",
        race(
            sequential(
                logged("Start delay", waitMs(startDelaySec * 1000.0)),
                race(
                    deadline(
                        drive.followCommand(outPath(), name = "Drive out"),
                        sequential(
                            waitUntil { drive.pathProgress() >= 0.5 },
                            instant { robot.recordEvent("AUTO: outbound midpoint") },
                        ),
                    ),
                    logged("Drive out time limit", waitMs(4_000.0)),
                ),
                drive.holdCommand(out, name = "Settle at out"),
                logged("Score", waitMs(300.0)), // stand-in for "score"
                drive.turnToCommand(alliance.mirror(Math.toRadians(90.0)), name = "Turn to 90 deg"),
                drive.followCommand(backPath(), holdEnd = true, name = "Drive back"),
                instant { robot.recordEvent("AUTO: complete") },
            ),
            logged("Routine time limit", waitMs(29_000.0)),
        ),
    )

    override fun onInitLoop() {
        if (driver.pressed(Button.DPAD_RIGHT)) startDelaySec = (startDelaySec + 1).coerceAtMost(10)
        if (driver.pressed(Button.DPAD_LEFT)) startDelaySec = (startDelaySec - 1).coerceAtLeast(0)
        telemetryBag.section("Auto") { put("start delay s", startDelaySec) }
    }

    override fun onStart() {
        if (!Constants.FORESIGHT_TUNED) {
            abortAuto("Foresight is not tuned")
            return
        }
        startedNs = robot.clock.nanos()
        waitingToStart = true
    }

    /** START pressed right after INIT: give the localizer a moment, then go anyway. */
    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            abortAuto("pose lost before start: $it")
            return
        }
        val waitedMs = (robot.clock.nanos() - startedNs) / 1e6
        if (!localizer.ready && waitedMs < LOCALIZER_WAIT_MS) return
        if (!localizer.ready) robot.recordEvent("AUTO: starting without a ready localizer: ${localizer.health()}")
        waitingToStart = false
        val selected = buildRoutine()
        routine = selected
        Scheduler.schedule(selected)
        if (!Scheduler.isScheduled(selected)) abortAuto("routine schedule rejected")
    }

    override fun onLoop() {
        if (waitingToStart) startRoutineWhenReady()
        val scheduled = routine?.let(Scheduler::isScheduled) ?: false
        telemetryBag.section("Auto") {
            put("alliance", alliance.name)
            put("pose", drive.pose)
            put("mode", drive.driveModeName)
            put("routine running", scheduled)
        }
        if (routine != null && !scheduled) {
            routine = null
            robot.recordEvent("AUTO: routine no longer scheduled; stopping")
            requestOpModeStop()
        }
    }

    private fun abortAuto(reason: String) {
        robot.recordEvent("AUTO ABORTED: $reason")
        requestOpModeStop()
    }

    private companion object {
        const val LOCALIZER_WAIT_MS = 1_000.0
    }
}
