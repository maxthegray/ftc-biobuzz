package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Drives the two-tip course with no mechanisms: a pathing test. RED only.
 *
 * Every path turns along its length (`.linear`). The robot settles in the
 * LOADING ZONE with a hold, then the op-mode stops.
 */
@Autonomous(name = "Just Path RED", group = "Diagnostics")
class JustPathAutoRed : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    // Picked on dpad left/right during init.
    private var startDelaySec = 0
    private var routine: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L

    override val initialAlliance: Alliance get() = Alliance.RED

    // Poses (x, y, heading°) in RED coordinates.
    private val poses get() = alliance.poses()
    private val start get() = poses.of(48.0, 9.0, 180.0)
    private val gardenSweep get() = poses.of(10.0, 10.0, 180.0)
    private val crossToFarSide get() = poses.of(48.0, 124.0, 90.0)
    // Not 129: the FLOWER stops the sensorbot at y ≈ 128.8, short of where Pedro ends the path.
    private val intoFarFlower get() = poses.of(48.0, 128.0, 90.0)
    private val parkInLoadingZone get() = poses.of(16.0, 106.0, 90.0)

    override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                onEvent = robot::recordEvent,
                isFollowing = drive::isFollowing,
                onFault = { routine?.let(Scheduler::cancel) },
                onStatusFault = {},
                startingPose = start,
            ),
        )
    }

    private fun gardenSweepPath(): Path = line(start, gardenSweep).linear(start, gardenSweep)

    private fun crossToFarSidePath(): Path = curve(
        gardenSweep,
        poses.of(26.0, 40.0, 0.0),
        poses.of(24.0, 108.0, 0.0),
        crossToFarSide,
    ).linear(gardenSweep, crossToFarSide)

    private fun intoFarFlowerPath(): Path = line(crossToFarSide, intoFarFlower).linear(crossToFarSide, intoFarFlower)

    private fun parkInLoadingZonePath(): Path = curve(
        intoFarFlower,
        poses.of(30.0, 126.0, 0.0),
        parkInLoadingZone,
    ).linear(intoFarFlower, parkInLoadingZone)

    private fun buildRoutine(): Command = logged(
        "Just Path routine",
        race(
            sequential(
                logged("Start delay", monotonicWaitMs(startDelaySec * 1000.0)),
                drive.followCommand(gardenSweepPath(), name = "Garden sweep"),
                drive.followCommand(crossToFarSidePath(), name = "Cross to far side"),
                drive.followCommand(intoFarFlowerPath(), name = "Into far FLOWER"),
                drive.followCommand(parkInLoadingZonePath(), name = "Park in LOADING ZONE"),
                drive.holdCommand(parkInLoadingZone, name = "Settle in LOADING ZONE"),
                instant { robot.recordEvent("AUTO: complete") },
            ),
            logged("Routine time limit", monotonicWaitMs(29_000.0)),
        ),
    )

    override fun onInitLoop() {
        if (driver.dpadRightPressed) startDelaySec = (startDelaySec + 1).coerceAtMost(10)
        if (driver.dpadLeftPressed) startDelaySec = (startDelaySec - 1).coerceAtLeast(0)
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

    /** START pressed right after INIT: give the localizer up to a second, then go anyway unless the pose is lost. */
    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            abortAuto("pose lost before start: $it")
            return
        }
        val waitedMs = (robot.clock.nanos() - startedNs) / 1e6
        if (!localizer.ready && waitedMs < 1_000.0) return
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
}
