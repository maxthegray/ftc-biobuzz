package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.Commands
import org.firstinspires.ftc.teamcode.core.pathing.path
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig

/**
 * Teleop for testing localization consistency over time.
 *
 * Behaves like [DriveOnlyTeleOp] during normal driving (including the
 * Back+Y heading reset and Back+B field-centric chords from [TeleOpBase]),
 * with two extra buttons:
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
class LocalizationTestTeleOp : TeleOpBase() {

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

    private var activeFollow: Command? = null
    private var targetLabel: String = "-"

    override fun configureTeleop() {
        // Y only when it isn't the Back+Y heading-reset chord; A only when
        // it isn't the Driver Station's Start+A gamepad re-bind chord.
        driver.trigger { driver.y && !driver.back }.onTrue(
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
                    robot.scheduler.cancel(it)
                    activeFollow = null
                    targetLabel = "-"
                }
            }.setName("stick interrupt").setPriority(CommandPriorities.DRIVER_ACTION),
        )
    }

    override fun onLoop() {
        activeFollow?.let {
            if (!robot.scheduler.isScheduled(it)) {
                activeFollow = null
                targetLabel = "-"
            }
        }
        emitTelemetry()
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /**
     * The path can only be built once the command actually runs (it starts at
     * the *current* pose), so this defers construction to schedule time. The
     * deferred follow handles its own interruption (breaks the follow);
     * [onLoop] clears the label once the command leaves the scheduler.
     */
    private fun followTo(label: () -> String, target: () -> Pose): Command {
        lateinit var outer: Command
        outer = Commands.defer(drive) {
            val start = drive.pose
            val chain = drive.path(startPose = start, alliance = alliance) {
                lineTo(target())
                constantHeading(start.heading)
            }
            activeFollow = outer
            targetLabel = label()
            drive.followCommand(chain, holdEnd = false)
        }
            .setName("followTo")
            .setPriority(CommandPriorities.DRIVER_ACTION)
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
