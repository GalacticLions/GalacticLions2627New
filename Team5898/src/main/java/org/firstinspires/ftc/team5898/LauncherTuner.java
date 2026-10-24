package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

/**
 * A testing file for launcher PIDF values.
 * <p>
 * gamepad1:
 * left trigger   speed up left launcher to targeted speed
 * right trigger  speed up right launcher to targeted speed
 *
 */

@TeleOp(name = "LauncherTuner")
public class LauncherTuner extends OpMode {
    private Robot robot;
    private TelemetryManager panels;
    double leftTarget = 0;
    double rightTarget = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.openGate();
    }

    @Override
    public void loop() {
        // Re-read PIDF so Panels edits take effect live
        robot.applyLauncherPIDF();

        // Full trigger = LAUNCHER_VELOCITY; snap it down for a step response

        if (gamepad1.left_trigger > 0.3) {
            leftTarget = Constants.TeleOp.LAUNCHER_VELOCITY;
        } else {
            leftTarget = 0;
        }

        if (gamepad1.right_trigger > 0.3) {
            rightTarget = Constants.TeleOp.LAUNCHER_VELOCITY;
        } else {
            rightTarget = 0;
        }

        robot.setLauncherVelocity(leftTarget, rightTarget);

        if (gamepad1.a) {
            robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
        } else {
            robot.setIntakePower(0);
        }

        // Graph target vs actual for each side
        panels.addData("leftTarget", leftTarget);
        panels.addData("leftVelocity", robot.getLauncherLeftVelocity());
        panels.addData("rightTarget", rightTarget);
        panels.addData("rightVelocity", robot.getLauncherRightVelocity());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.setLauncherVelocity(0, 0);
    }
}
