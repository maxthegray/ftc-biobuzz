package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/**
 * Pedro's mecanum drivetrain with voltage compensation, which Pedro 3 only has
 * for swerve. Every follower power (paths, holds, turns) is scaled by
 * {@code nominalVolts / battery volts}, so a power means the same motor effort on a
 * full battery and a tired one, and Foresight tuning holds across batteries.
 * Manual driving is not scaled: the sticks feel as before and full stick is
 * always full power.
 *
 * <p>The battery reading is smoothed over {@link #SMOOTHING_SECONDS}: under load
 * it swings by several volts within a tick. {@link #maxScaling} sees the scaled
 * powers too, so Foresight still knows when a wheel will saturate.
 */
public class VoltageCompensatedMecanum extends Mecanum {

    private static final double SMOOTHING_SECONDS = 0.5;

    private final double nominalVolts;
    private final DoubleSupplier batteryVolts;
    private double smoothedVolts = Double.NaN;
    private long lastReadNanos;

    public VoltageCompensatedMecanum(
            HardwareMap map, MecanumConfig config, double nominalVolts, DoubleSupplier batteryVolts) {
        super(map, config);
        this.nominalVolts = nominalVolts;
        this.batteryVolts = batteryVolts;
    }

    /** The multiplier on every follower power; 1 until the battery has been read. */
    public double scale() {
        return Double.isNaN(smoothedVolts) ? 1.0 : nominalVolts / smoothedVolts;
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        readBattery();
        super.drive(manual ? powers : scaled(powers), manual);
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return super.maxScaling(scaled(current), scaled(delta));
    }

    private DrivePowers scaled(DrivePowers powers) {
        double k = scale();
        return new DrivePowers(powers.forward() * k, powers.strafe() * k, powers.turn() * k);
    }

    private void readBattery() {
        double volts = batteryVolts.getAsDouble();
        // NaN before the first reading, 0 from a stopped hub: neither is the battery.
        if (!(volts > 0.0)) return;
        long now = System.nanoTime();
        if (Double.isNaN(smoothedVolts)) {
            smoothedVolts = volts;
        } else {
            double dt = (now - lastReadNanos) / 1e9;
            smoothedVolts += (volts - smoothedVolts) * (1.0 - Math.exp(-dt / SMOOTHING_SECONDS));
        }
        lastReadNanos = now;
    }
}
