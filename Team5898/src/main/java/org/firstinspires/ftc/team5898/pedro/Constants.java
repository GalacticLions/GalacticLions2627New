package org.firstinspires.ftc.team5898.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team5898.Hardware.RobotConstants;

public class Constants {
    // Set by auto, read by TeleOp. Statics survive between OpModes until the app restarts; null means no pose from auto
    public static Pose lastAutoPose = null;
    // Picked in auto's or TeleOp's init with A; shared so TeleOp keeps the alliance auto ran as
    public static boolean isRed = true;

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(RobotConstants.Hardware.MOTOR_FRONT_LEFT);
                c.frontRightName.set(RobotConstants.Hardware.MOTOR_FRONT_RIGHT);
                c.backRightName.set(RobotConstants.Hardware.MOTOR_BACK_RIGHT);
                c.backLeftName.set(RobotConstants.Hardware.MOTOR_BACK_LEFT);

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.000000000000000);
        c.yPodOffset.set(1.1250000000000000);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Run "Foresight Tuner" in AutoTune, then paste the body of the code block it prints here.
    // Until filled, Hold/Line/Curve tests throw "Config variable has not been set".
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2985424214088819);
                Controller secondaryTranslationalForward = Controller.proportional(0.1103034475101636);
                Controller primaryTranslationalLateral = Controller.proportional(0.4439842605032282);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16404032077800937);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019585005511039488));
                c.brake.set(Controller.proportionalFeedforward(0.016647254684383564));

                c.headingFeedback.set(Controller.proportional(4.570615120460551));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.040971892956805436, 0.010136617036034178));

                c.linearBrakeCoefficients.set(Matrix.diag(0.060241619390092635, 0.054364996238471));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002022150611486887, 0.0020512877099091945));

                c.maxAchievableForwardVelocity.set(54.684103928663276);
                c.maxAchievableStrafeVelocity.set(41.11092413815885);
                c.naturalForwardDeceleration.set(37.825488854410146);
                c.naturalStrafeDeceleration.set(76.66685254099323);
            }
    );
}
