package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team5898.Hardware.Robot;

// TODO docstring: list controls
@TeleOp(name = "NoHeadingHoldTeleOP")
public class NoHeadingHoldTeleOP extends OpMode {
    private Robot robot;
    private TelemetryManager panels;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        // Drive relative to the robot; stick Y is inverted. Turn stick goes straight to the motors, so any drift stays visible
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Graph heading in Panels to see how far the robot drifts
        panels.addData("heading", Math.toDegrees(robot.getHeadingRad()));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.drive(0, 0, 0);
    }
}
