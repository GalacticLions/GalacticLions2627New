package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

/**
 * A robot-centric TeleOp.
 * <p>
 * gamepad1 (driver):
 * left stick   drive / strafe
 * right stick  turn
 * <p>
 * gamepad2 (operator):
 * a            intake
 * b            shoot
 * x            stop everything
 * y            spit balls out (intake reversed)
 */

@TeleOp(name = "TeleOP")
public class TeleOP extends OpMode {
    private enum State {IDLE, INTAKING, OUTTAKING, SHOOTING}

    private Robot robot;
    private TelemetryManager panels;
    private State state = State.IDLE;
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime turnTimer = new ElapsedTime();
    private double holdHeadingDeg;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        // Hold whatever way auto left us facing, instead of snapping back to zero
        holdHeadingDeg = Math.toDegrees(robot.getHeadingRad());
    }

    @Override
    public void loop() {
        // Turn stick turns; once it's let go the IMU holds the heading, so strafing doesn't spin the robot
        double turn = gamepad1.right_stick_x;
        if (Math.abs(turn) > Constants.TeleOp.TURN_DEADBAND) {
            turnTimer.reset();
        }
        if (turnTimer.seconds() < Constants.TeleOp.HEADING_HOLD_DELAY_SEC) {
            holdHeadingDeg = Math.toDegrees(robot.getHeadingRad());
        } else {
            double error = AngleUnit.normalizeDegrees(holdHeadingDeg - Math.toDegrees(robot.getHeadingRad()));
            // IMU yaw is counterclockwise-positive but drive() turns clockwise, so flip the sign
            turn = Range.clip(-Constants.TeleOp.HEADING_KP * error,
                    -Constants.TeleOp.HEADING_MAX_POWER, Constants.TeleOp.HEADING_MAX_POWER);
        }

        // Drive relative to the robot; stick Y is inverted
        robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);

        // Re-read launcher PIDF so Panels edits take effect live
        robot.applyLauncherPIDF();

        // Operator buttons pick the state; x always wins
        if (gamepad2.xWasPressed()) {
            state = State.IDLE;
        } else if (gamepad2.aWasPressed()) {
            state = State.INTAKING;
        } else if (gamepad2.bWasPressed()) {
            state = State.SHOOTING;
            gateTimer.reset();
        } else if (gamepad2.yWasPressed()) {
            state = State.OUTTAKING;
        }

        // Each state sets every mechanism, so switching never leaves one running
        switch (state) {
            case IDLE:
                robot.setIntakePower(0);
                robot.setLauncherVelocity(0, 0);
                robot.closeGate();
                break;
            case INTAKING:
                robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
                robot.setLauncherVelocity(0, 0);
                robot.closeGate();
                break;
            case OUTTAKING:
                robot.setIntakePower(-Constants.TeleOp.INTAKE_POWER);
                robot.setLauncherVelocity(0, 0);
                robot.closeGate();
                break;
            case SHOOTING:
                robot.setLauncherVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_VELOCITY);
                robot.openGate();
                // Wait before feeding so the gate opens and the wheels spin up; a timer instead of sleep() so driving doesn't freeze
                if (gateTimer.seconds() >= Constants.TeleOp.GATE_OPEN_DELAY_SEC) {
                    robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
                } else {
                    robot.setIntakePower(0);
                }
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Launcher L/R", "%.0f / %.0f", robot.getLauncherLeftVelocity(), robot.getLauncherRightVelocity());
        telemetry.addData("At speed", robot.isLauncherAtVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_TOLERANCE));
        telemetry.addData("Gate", robot.getGatePosition());
        // Turning counterclockwise should make heading go up
        panels.addData("heading", Math.toDegrees(robot.getHeadingRad()));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        robot.drive(0, 0, 0);
        robot.setIntakePower(0);
        robot.setLauncherVelocity(0, 0);
    }
}
