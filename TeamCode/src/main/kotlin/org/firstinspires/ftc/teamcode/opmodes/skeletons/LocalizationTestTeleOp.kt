package org.firstinspires.ftc.teamcode.opmodes.skeletons

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.localization.shortestAngleDelta
import org.firstinspires.ftc.teamcode.opmodes.TeleOpBase
import org.firstinspires.ftc.teamcode.pedro.Constants

/**
 * Teleop for testing localization consistency over time.
 *
 * Behaves like Drive Only during normal driving (including the Back+Y
 * heading reset and Back+B field-centric chords from [TeleOpBase]), with two
 * extra buttons:
 *
 *  - **Triangle (Y)** — Pedro-follows to waypoint A (24" forward by default).
 *  - **Cross (A)** — Pedro-follows back to the test origin.
 *
 * Destinations and the path speed cap are configurable in Panels. Paths keep
 * the heading the robot had when the button was pressed; with
 * [turnToTargetHeading] the robot then turns to the target's heading. A
 * target closer than [COINCIDENT_WAYPOINT_TOLERANCE_INCHES] is not a path:
 * the robot only turns (if asked) or nothing happens. Press the active
 * target's button again, or move a stick, to cancel mid-move.
 * Afterwards control returns to manual teleop. Drive around, press again, and
 * compare where the robot thinks it ends up: that's localization drift.
 *
 * Disabled by default; needs Foresight tuning for the path buttons.
 */
@Disabled
@TeleOp(name = "Localization Test", group = "Diagnostics")
@Configurable
class LocalizationTestTeleOp : TeleOpBase() {

    companion object {
        /** X coordinate of waypoint A (inches). */
        @JvmField var waypointAX: Double = 24.0
        /** Y coordinate of waypoint A (inches). */
        @JvmField var waypointAY: Double = 0.0
        /** Heading at waypoint A (degrees), used with [turnToTargetHeading]. */
        @JvmField var waypointAHeadingDegrees: Double = 0.0

        /** X coordinate of the return target (inches). */
        @JvmField var returnX: Double = 0.0
        /** Y coordinate of the return target (inches). */
        @JvmField var returnY: Double = 0.0
        /** Heading at the return target (degrees), used with [turnToTargetHeading]. */
        @JvmField var returnHeadingDegrees: Double = 0.0

        /** Turn to the target's heading after arriving (or instead of a path, when already there). */
        @JvmField var turnToTargetHeading: Boolean = false

        /** Path speed cap as a fraction of the robot's max achievable velocity (Foresight maxPathSpeed). */
        @JvmField var pathSpeedFraction: Double = 0.3

        /** Stick axis magnitude above which a path is considered interrupted by the driver. */
        @JvmField var stickInterruptThreshold: Double = 0.1

        private const val DEFAULT_PATH_SPEED_FRACTION = 0.3

        /**
         * Inches. A target this close to the current position gets no path:
         * a zero-length Pedro line has no direction to follow.
         */
        const val COINCIDENT_WAYPOINT_TOLERANCE_INCHES = 0.25

        /**
         * The move from [start] to [target]'s position, keeping [start]'s
         * heading, then a turn to [targetHeading] (radians) when one is given
         * and differs by more than the hold tolerance. Null when there is
         * nothing to do.
         */
        internal fun moveCommand(
            drive: MecanumDriveSubsystem,
            start: Pose,
            target: Pose,
            targetHeading: Double?,
            pathSpeedFraction: Double,
        ): Command? {
            val steps = mutableListOf<Command>()
            if (start.distance(target) > COINCIDENT_WAYPOINT_TOLERANCE_INCHES) {
                val path = Paths.line(start, target)
                    .constant(start)
                    .with(Constants.foresightConfig.maxPathSpeed.at(pathSpeedFraction))
                steps += drive.followCommand(path, name = "Localization test path")
            }
            if (targetHeading != null &&
                abs(shortestAngleDelta(start.heading(), targetHeading)) > DriveConfig.safeHoldToleranceRadians
            ) {
                steps += drive.turnToCommand(targetHeading, name = "Localization test turn")
            }
            return when (steps.size) {
                0 -> null
                1 -> steps.single()
                else -> logged("Localization test move", sequential(*steps.toTypedArray()))
            }
        }
    }

    private var activeFollow: Command? = null
    private var activeButton: String? = null
    private var targetLabel: String = "-"
    private var lastPress: String = "-"

    override fun configureTeleop() {
        // Y only when it isn't the Back+Y heading-reset chord; A only when
        // it isn't the Driver Station's Start+A gamepad re-bind chord.
        driver.trigger { driver.y && !driver.back }.onTrue(
            moveOnPress("Y", { "(%.1f, %.1f)".format(waypointAX, waypointAY) }) {
                Pose(waypointAX, waypointAY, Math.toRadians(waypointAHeadingDegrees))
            },
        )
        driver.trigger { driver.a && !driver.start }.onTrue(
            moveOnPress("A", { "(%.1f, %.1f)".format(returnX, returnY) }) {
                Pose(returnX, returnY, Math.toRadians(returnHeadingDegrees))
            },
        )
        // Moving a stick takes the drive back at driver-action priority, which
        // interrupts the path through Ivy's requirements.
        driver.trigger { stickMoved() }.whileTrue(
            drive.teleopCommand(priority = CommandPriorities.DRIVER_ACTION, name = "Driver takeover") {
                TeleopInput(driver.leftStickY, driver.leftStickX, driver.rightStickX, driver.rightTrigger > 0.1)
            },
        )
    }

    override fun onLoop() {
        activeFollow?.let {
            if (!Scheduler.isScheduled(it)) {
                activeFollow = null
                activeButton = null
                targetLabel = "-"
            }
        }
        emitTelemetry()
    }

    /**
     * Decides at the press, from the *current* pose. The press itself requires
     * nothing, so a press with nothing to do leaves every running command alone.
     */
    private fun moveOnPress(button: String, label: () -> String, target: () -> Pose): Command = instant {
        val running = activeFollow?.takeIf(Scheduler::isScheduled)
        if (running != null && activeButton == button) {
            Scheduler.cancel(running)
            lastPress = "$button: cancelled"
            return@instant
        }
        val goal = target()
        val heading = if (turnToTargetHeading) goal.heading() else null
        val move = moveCommand(drive, drive.pose, goal, heading, safePathSpeedFraction())
        if (move == null) {
            lastPress = "$button: already at ${label()}"
            return@instant
        }
        Scheduler.schedule(move)
        if (Scheduler.isScheduled(move)) {
            activeFollow = move
            activeButton = button
            targetLabel = label()
            lastPress = "$button: moving"
        } else {
            lastPress = "$button: refused"
        }
    }

    private fun stickMoved(): Boolean =
        abs(driver.leftStickY) > stickInterruptThreshold ||
            abs(driver.leftStickX) > stickInterruptThreshold ||
            abs(driver.rightStickX) > stickInterruptThreshold

    private fun safePathSpeedFraction(): Double =
        if (pathSpeedFraction.isFinite() && pathSpeedFraction > 0.0) pathSpeedFraction.coerceAtMost(1.0) else DEFAULT_PATH_SPEED_FRACTION

    private fun emitTelemetry() {
        telemetryBag.section("Localization Test") {
            put("state", if (activeFollow == null) "TELEOP" else "PATH")
            put("target", targetLabel)
            put("last press", lastPress)
            put("fieldCentric", drive.fieldCentric)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.driveModeName)
        }
    }
}
