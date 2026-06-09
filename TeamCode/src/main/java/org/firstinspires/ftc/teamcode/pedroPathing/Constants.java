package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.runtime.RobotConfig;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * One source of truth for Pedro Pathing's physical + mechanical constants.
 *
 * Pedro reads this package via the exact name {@code pedroPathing.Constants} — it
 * must stay at this path. Everything here was tuned on our chassis; if you rebuild
 * the robot, re-run Pedro's forward/lateral and velocity tuners and paste the
 * numbers back  into this file.
 *
 * What lives where:
 *  - Physical constants (mass, zero-power accel, velocity caps, motor names + direction,
 *    Pinpoint pod offsets, encoder resolution): this file.
 *  - Hardware-map names (plain strings): also this file, kept in sync with
 *    {@link org.firstinspires.ftc.teamcode.core.runtime.RobotConfig}.
 *  - Runtime / game / driver feel knobs: {@link org.firstinspires.ftc.teamcode.core.subsystems.drive.DriveConfig}.
 */
public final class Constants {

    private Constants() {}

    /** Hardware-map name of the GoBilda Pinpoint driver. Exposed for tooling. */
    public static final String pinpointHardwareName = RobotConfig.Localization.PINPOINT;

    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(17.35)
            .forwardZeroPowerAcceleration(-63.2039)
            .lateralZeroPowerAcceleration(-56.000);

    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(51.42526)
            .yVelocity(53.52238)
            // Voltage compensation normalises wheel output to nominalVoltage so
            // paths stay consistent as the battery sags. xVelocity/yVelocity above
            // are voltage-dependent — re-run Pedro's velocity tuners with this
            // enabled, or the calibration point is just shifted, not fixed.
            .useVoltageCompensation(true)
            .nominalVoltage(12.0)
            .rightFrontMotorName(RobotConfig.Drive.FRONT_RIGHT_MOTOR)
            .rightRearMotorName(RobotConfig.Drive.BACK_RIGHT_MOTOR)
            .leftRearMotorName(RobotConfig.Drive.BACK_LEFT_MOTOR)
            .leftFrontMotorName(RobotConfig.Drive.FRONT_LEFT_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.372)
            .strafePodX(-6.441)
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .hardwareMapName(pinpointHardwareName)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Build a fully-configured {@link Follower} with mecanum drive + Pinpoint
     * localisation. Call this exactly once per op-mode (typically from the
     * {@code configure()} hook on the starter's {@code OpModeBase}).
     *
     * <p>The Pinpoint is read synchronously inside {@code Follower.update()}.
     * Moving that I2C read to a background thread was tried and reverted: the
     * Pinpoint shares the Control Hub's Lynx serial link with the drive-motor
     * writes, so a background poll just starves on link contention (~12–25 ms
     * per read) while the main loop spins on a stale pose. The synchronous read
     * keeps localisation in lockstep with the control loop — that is the right
     * trade even though it caps loop rate around the Pinpoint read cost.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    /**
     * Same follower, but with a caller-supplied localizer instead of the
     * direct-I2C Pinpoint — e.g. {@code SRSHubPinpointLocalizer} when the
     * Pinpoint hangs off an SRSHub. Note the Follower's constructor calls
     * {@code localizer.resetIMU()} immediately, so the localizer must
     * tolerate being poked before the op-mode initialises its hardware.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(localizer)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
