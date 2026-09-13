package org.firstinspires.ftc.teamcode.opmodes.skeletons

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.lazy
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
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
 * Destinations and the path speed cap are configurable in Panels. Press the
 * active target's button again, or move a stick, to cancel mid-path.
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

        /** X coordinate of the return target (inches). */
        @JvmField var returnX: Double = 0.0
        /** Y coordinate of the return target (inches). */
        @JvmField var returnY: Double = 0.0

        /** Path speed cap as a fraction of the robot's max achievable velocity (Foresight maxPathSpeed). */
        @JvmField var pathSpeedFraction: Double = 0.3

        /** Stick axis magnitude above which a path is considered interrupted by the driver. */
        @JvmField var stickInterruptThreshold: Double = 0.1

        private const val DEFAULT_PATH_SPEED_FRACTION = 0.3
    }

    private var activeFollow: Command? = null
    private var targetLabel: String = "-"

    override val restorePoseFromAuton: Boolean get() = false

    override fun configureTeleop() {
        // Y only when it isn't the Back+Y heading-reset chord; A only when
        // it isn't the Driver Station's Start+A gamepad re-bind chord.
        driver.trigger { driver.y && !driver.back }.toggleOnTrue(
            followTo(label = { "(%.1f, %.1f)".format(waypointAX, waypointAY) }) { Pose(waypointAX, waypointAY) },
        )
        driver.trigger { driver.a && !driver.start }.toggleOnTrue(
            followTo(label = { "(%.1f, %.1f)".format(returnX, returnY) }) { Pose(returnX, returnY) },
        )
        // Moving a stick takes the drive back at driver-action priority, which
        // interrupts the path through Ivy's requirements.
        driver.trigger { stickMoved() }.whileTrue(
            drive.teleopCommand(priority = CommandPriorities.DRIVER_ACTION) {
                TeleopInput(driver.leftStickY, driver.leftStickX, driver.rightStickX, driver.rightTrigger > 0.1)
            },
        )
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

    /** The path starts at the *current* pose, so it is built when the command starts. */
    private fun followTo(label: () -> String, target: () -> Pose): Command {
        lateinit var outer: Command
        outer = lazy {
            val start = drive.pose
            val path = Paths.line(start, target())
                .constant(start)
                .with(Constants.foresightConfig.maxPathSpeed.at(safePathSpeedFraction()))
            activeFollow = outer
            targetLabel = label()
            drive.followCommand(path)
        }
            .requiring(drive)
            .setPriority(CommandPriorities.DRIVER_ACTION)
        return outer
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
            put("fieldCentric", drive.fieldCentric)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.driveModeName)
        }
    }
}
