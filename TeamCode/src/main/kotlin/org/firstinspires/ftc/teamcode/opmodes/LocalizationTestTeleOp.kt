package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.commands.Commands
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

/**
 * Teleop for testing localization consistency over time.
 *
 * Behaves like [DriveOnlyTeleOp] during normal driving, with two extra buttons:
 *
 *  - **Triangle (Y)** — Pedro-follows to waypoint A (configurable in Panels).
 *    Press again, or move a stick, or press Cross to cancel mid-path.
 *  - **Cross (A)** — Pedro-follows to field center. Same interrupt rules.
 *
 * After each path (completed or interrupted) control returns to manual teleop
 * automatically. Drive around, press again, and compare where the robot thinks
 * it ends up — that's the localization drift you're measuring.
 *
 * All destination coordinates are live-editable in Panels (see companion object).
 */
@TeleOp(name = "Starter: Localization Test", group = "Starter")
@Configurable
class LocalizationTestTeleOp : OpModeBase() {

    companion object {
        /** X coordinate of waypoint A (inches). Adjust for your field. */
        @JvmField var waypointAX: Double = 70.75
        /** Y coordinate of waypoint A (inches). Adjust for your field. */
        @JvmField var waypointAY: Double = 70.75

        /** X coordinate of the field-center target (inches). */
        @JvmField var fieldCenterX: Double = 70.75
        /** Y coordinate of the field-center target (inches). */
        @JvmField var fieldCenterY: Double = 70.75

        /** Stick axis magnitude above which a path is considered interrupted by the driver. */
        @JvmField var stickInterruptThreshold: Double = 0.1
    }

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private var activeFollow: Command? = null
    private var targetLabel: String = "-"

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(LocalizerSubsystem(follower))
        drive.defaultCommand = drive.teleopCommand {
            TeleopInput(
                forward = driver.leftStickY,
                strafe = driver.leftStickX,
                turn = driver.rightStickX,
                precision = driver.rightTrigger > 0.1,
            )
        }
        (driver.button(Button.START) and driver.button(Button.A))
            .onTrue(
                Commands.instant { localizer.setPose(drive.pose.withHeading(0.0)) }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
        (driver.button(Button.BACK) and driver.button(Button.B))
            .onTrue(
                Commands.instant { DriveConfig.fieldCentric = !DriveConfig.fieldCentric }
                    .setPriority(CommandPriorities.DRIVER_ACTION),
            )
        driver.button(Button.Y).onTrue(
            followTo(
                label = { "(%.1f, %.1f)".format(waypointAX, waypointAY) },
                target = { Pose(waypointAX, waypointAY, 0.0) },
            ),
        )
        driver.trigger { driver.a && !driver.start }.onTrue(
            followTo(
                label = { "(%.1f, %.1f)".format(fieldCenterX, fieldCenterY) },
                target = { Pose(fieldCenterX, fieldCenterY, 0.0) },
            ),
        )
        driver.trigger { stickMoved() }.whileTrue(
            Commands.infinite {
                activeFollow?.let {
                    Scheduler.cancel(it)
                    activeFollow = null
                    targetLabel = "-"
                }
            }.setPriority(CommandPriorities.DRIVER_ACTION),
        )
    }

    override fun onStart() {
        localizer.restorePersistedPose()
    }

    override fun onLoop() {
        activeFollow?.let {
            if (!Scheduler.isScheduled(it)) {
                activeFollow = null
                targetLabel = "-"
            }
        }
        emitTelemetry()
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private fun followTo(label: () -> String, target: () -> Pose): Command {
        var child: Command? = null
        lateinit var outer: Command
        outer = Command.build()
            .requiring(drive)
            .setPriority(CommandPriorities.DRIVER_ACTION)
            .setStart {
                val start = drive.pose
                val chain = drive.path(startPose = start, alliance = alliance) {
                    lineTo(target())
                    constantHeading(start.heading)
                }
                child = drive.followCommand(chain, holdEnd = false)
                activeFollow = outer
                targetLabel = label()
                child?.start()
            }
            .setExecute { child?.execute() }
            .setDone { child?.done() ?: true }
            .setEnd { endCondition ->
                child?.end(endCondition)
                if (activeFollow === outer) {
                    activeFollow = null
                    targetLabel = "-"
                }
                if (endCondition == EndCondition.INTERRUPTED) drive.breakPath()
            }
        return outer
    }

    private fun stickMoved(): Boolean =
        abs(driver.leftStickY) > stickInterruptThreshold ||
            abs(driver.leftStickX) > stickInterruptThreshold ||
            abs(driver.rightStickX) > stickInterruptThreshold

    private fun emitTelemetry() {
        val state = if (activeFollow == null) "TELEOP" else "PATH"
        telemetryBag.section("Localization Test") {
            put("state", state)
            put("target", targetLabel)
            put("fieldCentric", DriveConfig.fieldCentric)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
        }
    }
}
