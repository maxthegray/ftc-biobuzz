package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Skeleton of the two-tip auto (route: `BiobuzzRedAuto.pp` in the Pedro
 * visualizer). RED coordinates; BLUE is the season's 180° rotation.
 *
 * Drives the real route; every mechanism step is a named [standIn] wait.
 * Kept this simple on purpose until TeleOp works and the robot is built.
 */
@Disabled
@Autonomous(name = "Two Tip RED", group = "Match")
open class TwoTipAutoRed : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private var routine: Command? = null

    override val initialAlliance: Alliance get() = Alliance.RED

    // Poses (x, y, heading°) in RED coordinates.
    private val poses get() = alliance.poses()
    private val start get() = poses.of(48.0, 9.0, 180.0)
    private val gardenEnd get() = poses.of(10.0, 10.0, 180.0)
    private val farShot get() = poses.of(48.0, 124.0, 90.0)
    private val flower get() = poses.of(48.0, 129.0, 90.0)
    private val park get() = poses.of(16.0, 106.0, 90.0)

    override fun configure() {
        val follower = Constants.create(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                isFollowing = drive::isFollowing,
                onFault = { routine?.let(Scheduler::cancel) },
                startingPose = start,
            ),
        )
    }

    /** A mechanism step that doesn't exist yet: waits [ms] and shows up in the log under [name]. */
    private fun standIn(name: String, ms: Double): Command = logged(name, monotonicWaitMs(ms))

    // Planned, not built yet:
    //  - The turret tracks the HIVE all match as its default command, so the
    //    Limelight always sees which CELL is raised.
    //  - Wrap everything before parking in a race with a 25 s timer, then park
    //    from wherever the robot is (lazy path from drive.pose).
    //  - Wrap the whole routine in a race with a 29 s timer.
    private fun buildRoutine(): Command = sequential(
        // Fire all 4 preloads at the starting CELL; it already holds 3 NECTAR, so 3 POLLEN tip it.
        // Later: aim the turret, spin the flywheel to ShotModel speed, feed until empty.
        standIn("Fire preloads", 2000.0),

        // Sweep the GARDEN along the audience wall with the intake running.
        // Later: race the path with "intake full" (needs a ball sensor) so it stops at 4 balls.
        drive.followCommand(line(start, gardenEnd).constant(start)),
        standIn("Intake GARDEN", 500.0),

        // Later: wait up to 1 s for the camera to see the HIVE, then branch:
        //  - starting CELL still raised (preloads didn't tip it): fire the GARDEN load at it from
        //    here, then cross;
        //  - tipped, or never seen: cross, then fire at the far CELL (what runs now).
        drive.followCommand(
            curve(gardenEnd, poses.of(26.0, 40.0, 0.0), poses.of(24.0, 108.0, 0.0), farShot).linear(gardenEnd, farShot),
            holdEnd = true,
        ),
        standIn("Fire GARDEN load", 2000.0),

        // Drive into the far-wall FLOWER and empty its 4 POLLEN into the intake.
        // Later: the FLOWER mechanism, with a time limit so a jam can't eat the auto.
        drive.followCommand(line(farShot, flower).constant(flower), holdEnd = true),
        standIn("Empty FLOWER", 2500.0),

        // Fire from the same spot at the far CELL: 8 POLLEN in total there, so tip #2.
        standIn("Fire FLOWER load", 2000.0),

        // Park partly in the LOADING ZONE without touching the wall (PARK + LEAVE).
        drive.followCommand(curve(flower, poses.of(30.0, 126.0, 0.0), park).constant(flower), holdEnd = true),
    )

    override fun onStart() {
        if (!Constants.FORESIGHT_TUNED || !localizer.ready) {
            requestOpModeStop()
            return
        }
        routine = buildRoutine().also(Scheduler::schedule)
    }

    override fun onLoop() {
        if (routine?.let(Scheduler::isScheduled) == false) requestOpModeStop()
    }
}

@Disabled
@Autonomous(name = "Two Tip BLUE", group = "Match")
class TwoTipAutoBlue : TwoTipAutoRed() {
    override val initialAlliance: Alliance get() = Alliance.BLUE
}
