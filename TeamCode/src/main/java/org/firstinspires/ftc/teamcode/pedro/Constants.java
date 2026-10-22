package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveConfig;

/**
 * Pedro Pathing 3 constants for this robot, in the layout AutoTune generates
 * (https://pedropathing.com/docs/pathing/tuning/constants). Paste AutoTune's
 * Java output over the matching block; keep the hardware names pointing at
 * {@link RobotConfig}.
 *
 * <p>Provenance of every value:
 * <ul>
 *   <li>Motor names: {@link RobotConfig.Drive}. Motor directions: carried from the
 *       Pedro 2.1.1 constants (FL/BL reversed). Re-check with AutoTune's Mecanum
 *       procedure on the competition chassis.</li>
 *   <li>Pinpoint offsets and directions: AutoTune's Pinpoint procedure on the
 *       sensorbot, copied from 23209-biobuzz (commit ba7c04f). They replace the
 *       CAD offsets (x 2.8346 in, y 0; commit 4775780), which disagree in sign;
 *       the tuner can flip a pod's sign (Pedro issue #185).</li>
 *   <li>Foresight: AutoTune's Foresight procedure on the sensorbot with those
 *       offsets, copied from 23209-biobuzz (commit 1776795, tuned 2026-10-21).
 *       Re-run AutoTune on the competition chassis.</li>
 * </ul>
 *
 * <p>Pedro 2's voltage compensation has no Pedro 3 equivalent.
 */
public final class Constants {

    private Constants() {}

    /**
     * True once {@link #foresightConfig} holds AutoTune output. While false the
     * follower has no path algorithm: manual driving works, and path, hold and turn
     * commands refuse to start.
     */
    public static final boolean FORESIGHT_TUNED = true;

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.Drive.FRONT_LEFT_MOTOR);
        c.backLeftName.set(RobotConfig.Drive.BACK_LEFT_MOTOR);
        c.frontRightName.set(RobotConfig.Drive.FRONT_RIGHT_MOTOR);
        c.backRightName.set(RobotConfig.Drive.BACK_RIGHT_MOTOR);

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.Localization.PINPOINT);
        c.xPodOffset.set(-3.220380347544753);
        c.yPodOffset.set(0.717855813935047);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // No device command at construction. LocalizerSubsystem writes the start
        // pose at every INIT instead. RECALIBRATE_IMU (Pedro's default) and
        // RESET_AND_RECALIBRATE_IMU would recalibrate the gyro every run, which
        // needs a stationary robot for ~0.25 s and is not needed to zero the pose.
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    /** AutoTune's Foresight output (sensorbot, 2026-10-21). */
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3859305130404934);
        Controller secondaryTranslationalForward = Controller.proportional(0.14259101231523041);
        Controller primaryTranslationalLateral = Controller.proportional(0.649893858954874);
        Controller secondaryTranslationalLateral = Controller.proportional(0.24011841540008994);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.008317315818631975));
        c.brake.set(Controller.proportionalFeedforward(0.007069718445837179));

        c.headingFeedback.set(Controller.proportional(7.0912011875081555));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.10744791340779461, 0.006264631361127194));

        c.linearBrakeCoefficients.set(Matrix.diag(0.18552051491361324, 0.10012107419859242));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002071194869299692, 0.0022879444427973015));

        c.maxAchievableForwardVelocity.set(68.3251044222146);
        c.maxAchievableStrafeVelocity.set(61.181111299438065);
        c.naturalForwardDeceleration.set(41.270915401949004);
        c.naturalStrafeDeceleration.set(64.36119081305444);
    });

    /**
     * Builds the follower: Pinpoint localizer, mecanum drivetrain, and Foresight
     * once tuned. Call once per op-mode.
     *
     * <p>The Pinpoint is read synchronously inside {@code Follower.update()}.
     * Moving that I2C read to a background thread was tried and reverted: the
     * Pinpoint shares the Control Hub's Lynx serial link with the drive-motor
     * writes, so a background poll starves on link contention (~12–25 ms per
     * read) while the main loop spins on a stale pose.
     */
    public static Follower create(HardwareMap hardwareMap) {
        drivetrainConfig.manualBrakeMode.set(DriveConfig.brakeOnTeleop);
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                FORESIGHT_TUNED ? new Foresight(foresightConfig) : null
        );
    }
}
