package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * One source of truth for Pedro Pathing's physical + mechanical constants.
 *
 * Pedro reads this package via the exact name {@code pedroPathing.Constants} — it
 * must stay at this path. Everything here was tuned on our chassis; if you rebuild
 * the robot, re-run Pedro's forward/lateral and velocity tuners and paste the
 * numbers back into this file.
 *
 * What lives where:
 *  - Physical constants (mass, zero-power accel, velocity caps, motor names + direction,
 *    Pinpoint pod offsets, encoder resolution): this file.
 *  - Hardware-map names (plain strings): also this file, kept in sync with
 *    {@link org.firstinspires.ftc.teamcode.starter.config.RobotConfig}.
 *  - Runtime / game / driver feel knobs: {@link org.firstinspires.ftc.teamcode.starter.drive.DriveConfig}.
 */
public final class Constants {

    private Constants() {}

    /** Hardware-map name of the GoBilda Pinpoint driver. Exposed for tooling. */
    public static final String pinpointHardwareName = "sensor_otos";

    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(17.35)
            .forwardZeroPowerAcceleration(-63.2039)
            .lateralZeroPowerAcceleration(-56.000);

    public static final MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(51.42526)
            .yVelocity(53.52238)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
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
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
