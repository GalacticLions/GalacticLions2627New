package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.team5898.pedro.Constants;
import org.firstinspires.ftc.team5898.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.team5898.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.team5898.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.team5898.pedro.procedures.Tests;


public class Tuning {
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
        return new ForesightTuner(h -> new PinpointLocalizer(h, Constants.localizerConfig), h -> new Mecanum(h, Constants.driveConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareDevices -> new Mecanum(hardwareDevices, Constants.driveConfig), (hardwareDevices -> new PinpointLocalizer(hardwareDevices, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
