package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig;
import org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig;

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
 *   <li>Pinpoint offsets and directions: measured on the sensorbot (CAD offsets,
 *       commit 4775780; frame redefinition and encoder directions confirmed on the
 *       hardware, commit 2f13376). Pedro 2's forwardPodY/strafePodX map to
 *       xPodOffset/yPodOffset: both versions pass them to
 *       {@code GoBildaPinpointDriver.setOffsets(x, y)} in that order.</li>
 *   <li>Foresight: <b>not tuned</b>. Pedro 2's mass, zero-power accelerations and
 *       velocities came from the starter scaffold, were never measured on this
 *       chassis, and have no Foresight equivalent. Run AutoTune's Foresight
 *       procedure, paste its output into {@link #foresightConfig}, then set
 *       {@link #FORESIGHT_TUNED} to true.</li>
 * </ul>
 *
 * <p>Pedro 2's voltage compensation has no Pedro 3 equivalent.
 */
public final class Constants {

    private Constants() {}

    /**
     * False until {@link #foresightConfig} holds AutoTune output. While false the
     * follower has no path algorithm: manual driving works, and path, hold and turn
     * commands refuse to start.
     */
    public static final boolean FORESIGHT_TUNED = false;

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
        // Both pods sit on the lateral centreline, 72 mm (2.8346 in) either side of
        // the robot centre. The forward (X) pod is on the robot's left, so its
        // lateral offset is positive; the strafe (Y) pod has no fore/aft offset.
        c.xPodOffset.set(2.8346);
        c.yPodOffset.set(0.0);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    /** Paste AutoTune's Foresight output here. Required values are deliberately unset. */
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
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
