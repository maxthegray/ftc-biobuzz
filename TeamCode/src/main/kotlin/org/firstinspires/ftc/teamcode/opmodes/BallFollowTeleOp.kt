package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.command.Command
import org.firstinspires.ftc.teamcode.core.command.EndCondition
import org.firstinspires.ftc.teamcode.core.runtime.CommandPriorities
import org.firstinspires.ftc.teamcode.core.runtime.ConfigStore
import org.firstinspires.ftc.teamcode.core.runtime.Preflight
import org.firstinspires.ftc.teamcode.core.subsystems.drive.MecanumDriveSubsystem.TeleopInput
import org.firstinspires.ftc.teamcode.core.subsystems.vision.LimelightSubsystem
import org.firstinspires.ftc.teamcode.core.util.GamepadEx.Button
import org.firstinspires.ftc.teamcode.vision.BallAimConfig
import org.firstinspires.ftc.teamcode.vision.BallAimController

/**
 * Drive plus a Limelight ball-heading assist — an ordinary [TeleOpBase] teleop,
 * built the way a season teleop should be: subsystems and trigger bindings in
 * [configureTeleop], telemetry in [onLoop], nothing imperative in the loop.
 * Copy this shape for future teleops.
 *
 * Controls (the [TeleOpBase] standard set plus this op-mode's assist):
 *  - left stick translate, right stick turn
 *  - **right trigger** — precision mode while held
 *  - **right bumper** — aim at the yellow ball while held; the Limelight owns
 *    heading, the driver keeps translation
 *  - **Back + Y** — reset heading, **Back + B** — toggle field-centric
 */
@TeleOp(name = "Ball Follow", group = "Match")
class BallFollowTeleOp : TeleOpBase() {

    private lateinit var limelight: LimelightSubsystem
    private val ballAim = BallAimController()

    /** Nanos of the previous aim step, for the controller's dt. */
    private var lastAimNs = Long.MIN_VALUE

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard +
            Preflight.Requirement(LimelightSubsystem.DEFAULT_HARDWARE_NAME, Limelight3A::class.java)

    override fun configureTeleop() {
        ConfigStore.register("ballAim", BallAimConfig)
        // Pipeline 0 is the subsystem's own default; the yellow color config
        // lives on the Limelight, not here.
        limelight = robot.register(LimelightSubsystem())

        driver.button(Button.RIGHT_BUMPER).whileTrue(aimAtBallCommand())
    }

    /**
     * Hands the turn channel to the Limelight while translation still runs
     * through the normal teleop path — same input curve, power scale, precision
     * trigger, and field-centric selection the driver already has.
     *
     * At [CommandPriorities.AUTON_ROUTINE] this preempts the drive's default
     * teleop command (which resumes on release, on its own) but still loses to
     * driver actions like the Back+Y heading reset.
     */
    private fun aimAtBallCommand(): Command = drive.teleopCommand(
        name = "aim at ball",
        priority = CommandPriorities.AUTON_ROUTINE,
        onStart = {
            ballAim.reset()
            lastAimNs = Long.MIN_VALUE
            robot.recordEvent("BALL AIM: engaged")
        },
        onEnd = { endCondition: EndCondition ->
            ballAim.reset()
            robot.recordEvent("BALL AIM: released ($endCondition)")
        },
    ) {
        TeleopInput(
            forward = driver.leftStickY,
            strafe = driver.leftStickX,
            turn = 0.0,
            precision = driver.rightTrigger > 0.1,
            turnPower = ballAim.update(aimDtSeconds(), selectedTx()),
        )
    }

    private fun selectedTx(): Double? =
        ballAim.selectTx(limelight.colorTargets, limelight.primaryTarget)

    private fun aimDtSeconds(): Double {
        val now = robot.clock.nanos()
        val previous = lastAimNs
        lastAimNs = now
        // First tick after engaging has no interval to measure; a zero dt makes
        // PIDFController skip its derivative term rather than divide by it.
        return if (previous == Long.MIN_VALUE) 0.0 else (now - previous) / 1e9
    }

    override fun onLoop() {
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("mode", drive.mode.name)
            put("fieldCentric", drive.fieldCentric)
        }
        telemetryBag.section("Ball Aim") {
            put("limelight", limelight.health())
            put("targets", limelight.targetCount)
            put("has target", ballAim.hasTarget)
            put("on target", ballAim.onTarget)
            put("tx deg", ballAim.lastTxDegrees, decimals = 2)
            put("turn power", ballAim.lastOutput, decimals = 3)
        }
    }
}
