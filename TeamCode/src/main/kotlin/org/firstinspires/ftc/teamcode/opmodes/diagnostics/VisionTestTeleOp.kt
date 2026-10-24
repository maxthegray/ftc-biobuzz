package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.vision.GoalAim

/**
 * Stationary Limelight check; commands no motors. Shows every tag in view and
 * the goal [GoalAim] picks for each alliance. Use it to confirm that `ty` grows
 * as a tag moves up the image, and whether a lowered CELL's tags are ever seen.
 */
@TeleOp(name = "Vision Test", group = "Diagnostics")
class VisionTestTeleOp : OpModeBase() {

    private lateinit var limelight: LimelightSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(RobotConfig.Vision.LIMELIGHT, Limelight3A::class.java))
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        limelight = robot.register(LimelightSubsystem())
    }

    override fun onInitLoop() = show()

    override fun onLoop() = show()

    private fun show() {
        telemetryBag.section("Limelight") {
            put("status", limelight.health())
            put("tags", limelight.tags.sortedBy { it.id }.joinToString("\n") { "${it.id}: tx ${it.txDeg.f()} ty ${it.tyDeg.f()}" })
            for (alliance in Alliance.entries) {
                val goal = GoalAim.goal(alliance, limelight.tags)
                put("$alliance goal", goal?.let { "tags ${it.tagIds} tx ${it.txDeg.f()} ty ${it.tyDeg.f()}" } ?: "none")
            }
        }
    }

    private fun Double.f() = String.format(Locale.US, "%.1f", this)
}
