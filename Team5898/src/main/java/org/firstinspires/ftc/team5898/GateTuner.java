package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

// controls: gamepad1 dpad up +GATE_STEP, dpad down -GATE_STEP
@TeleOp(name = "GateTuner")
public class GateTuner extends OpMode {
    private Robot robot;
    private TelemetryManager panels;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        // Step the gate one press at a time; it doesn't move until a dpad is pressed
        double step = 0;
        if (gamepad1.dpadUpWasPressed()) {
            step = Constants.Gate.GATE_STEP;
        } else if (gamepad1.dpadDownWasPressed()) {
            step = -Constants.Gate.GATE_STEP;
        }
        if (step != 0) {
            // Nothing has been commanded yet on the first press, so start from closed
            double current = robot.getGatePosition();
            if (Double.isNaN(current)) {
                current = Constants.Gate.GATE_CLOSED;
            }
            robot.setGatePosition(Range.clip(current + step, 0.0, 1.0));
        }

        // Servos have no feedback, so this is the last commanded position
        panels.addData("gatePosition", robot.getGatePosition());
        panels.update(telemetry);
    }
}
