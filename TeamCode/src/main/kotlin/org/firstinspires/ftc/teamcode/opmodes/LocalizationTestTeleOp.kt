package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.core.runtime.OpModeBase
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.core.subsystems.localization.Localizer
import org.firstinspires.ftc.teamcode.core.pathing.path

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
    private lateinit var localizer: Localizer

    private enum class State { TELEOP, PATH_TO_WAYPOINT_A, PATH_TO_CENTER }
    private var state = State.TELEOP

    override fun configure() {
        val follower = Constants.createFollower(hardwareMap)
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(Localizer(follower))
    }

    override fun onStart() {
        drive.enableTeleop()
    }

    override fun onLoop() {
        when (state) {
            State.TELEOP -> {
                // Standard drive controls (mirrors DriveOnlyTeleOp).
                if (driver.start && driver.aPressed) {
                    localizer.setPose(drive.pose.withHeading(0.0))
                }
                if (driver.back && driver.bPressed) {
                    DriveConfig.fieldCentric = !DriveConfig.fieldCentric
                }

                val precision = driver.rightTrigger > 0.1
                drive.drive(
                    forward = driver.leftStickY,
                    strafe = driver.leftStickX,
                    turn = driver.rightStickX,
                    precision = precision,
                )

                // Path triggers — must come after the drive call so stick reads are fresh.
                if (driver.yPressed) {
                    goTo(Pose(waypointAX, waypointAY, 0.0), State.PATH_TO_WAYPOINT_A)
                }
                if (driver.aPressed) {
                    goTo(Pose(fieldCenterX, fieldCenterY, 0.0), State.PATH_TO_CENTER)
                }
            }

            State.PATH_TO_WAYPOINT_A, State.PATH_TO_CENTER -> {
                if (shouldInterrupt()) {
                    cancelPath()
                } else if (!drive.isFollowing) {
                    // Path completed naturally — hand control back to the driver.
                    drive.enableTeleop()
                    state = State.TELEOP
                }
            }
        }

        emitTelemetry()
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private fun goTo(target: Pose, newState: State) {
        val chain = drive.path(startPose = drive.pose) {
            lineTo(target)
            constantHeading(drive.pose.heading)
        }
        drive.followPath(chain, holdEnd = false)
        state = newState
    }

    private fun shouldInterrupt(): Boolean {
        val stickMoved = abs(driver.leftStickY)  > stickInterruptThreshold
                      || abs(driver.leftStickX)  > stickInterruptThreshold
                      || abs(driver.rightStickX) > stickInterruptThreshold
        return stickMoved || driver.yPressed || driver.aPressed
    }

    private fun cancelPath() {
        drive.breakPath()
        drive.enableTeleop()
        state = State.TELEOP
    }

    private fun emitTelemetry() {
        val targetLabel = when (state) {
            State.PATH_TO_WAYPOINT_A -> "(%.1f, %.1f)".format(waypointAX, waypointAY)
            State.PATH_TO_CENTER   -> "(%.1f, %.1f)".format(fieldCenterX, fieldCenterY)
            State.TELEOP           -> "—"
        }
        telemetryBag.section("Localization Test") {
            put("state", state.name)
            put("target", targetLabel)
            put("fieldCentric", DriveConfig.fieldCentric)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("velocity", drive.velocity)
            put("mode", drive.mode.name)
        }
        telemetryBag.section("Robot") {
            put("loopHz", robot.loopHz, decimals = 1)
            put("loopCount", robot.loopCount)
        }
    }
}
