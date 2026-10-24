package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * AutoTune procedures, served at http://192.168.43.1:10158 while connected to
 * the Robot Controller. The procedures in {@code procedures/} are copied
 * unmodified from Pedro-Pathing/Quickstart at b4312385; only the Mecanum and
 * Pinpoint ones this robot uses are included. Foresight and Tests drive through
 * the same voltage compensation as the op-modes, so the tuning matches.
 */
public final class Tuning {

    private Tuning() {}

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                Tuning::drivetrain
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                Tuning::drivetrain,
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                Constants.FORESIGHT_TUNED ? () -> new Foresight(Constants.foresightConfig) : null
        );
    }

    private static VoltageCompensatedMecanum drivetrain(HardwareMap hardwareMap) {
        return Constants.drivetrain(hardwareMap, hardwareMap.voltageSensor.iterator().next()::getVoltage);
    }
}
