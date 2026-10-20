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

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        // Re-read PIDF so Panels edits take effect live
        robot.applyLauncherPIDF();

        // Full trigger = LAUNCHER_VELOCITY; snap it down for a step response
        double leftTarget = gamepad1.left_trigger * Constants.TeleOp.LAUNCHER_VELOCITY;
        double rightTarget = gamepad1.right_trigger * Constants.TeleOp.LAUNCHER_VELOCITY;
        robot.setLauncherVelocity(leftTarget, rightTarget);

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
