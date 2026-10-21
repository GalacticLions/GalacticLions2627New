package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

/**
 * A field-centric TeleOp.
 * <p>
 * gamepad1 (driver):
 * left stick   drive / strafe
 * right stick  turn
 * guide        reset heading
 * <p>
 * gamepad2 (operator):
 * a            intake
 * b            shoot
 * x            stop everything
 */

@TeleOp(name = "TeleOP")
public class TeleOP extends OpMode {
    private enum State {IDLE, INTAKING, SHOOTING}

    private Robot robot;
    private State state = State.IDLE;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        // Reset heading so the current facing becomes "forward"
        if (gamepad1.guide) {
            robot.resetHeading();
        }

        // Drive relative to the field; stick Y is inverted
        robot.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Re-read launcher PIDF so Panels edits take effect live
        robot.applyLauncherPIDF();

        // Operator buttons pick the state; x always wins
        if (gamepad2.xWasPressed()) {
            state = State.IDLE;
        } else if (gamepad2.aWasPressed()) {
            state = State.INTAKING;
        } else if (gamepad2.bWasPressed()) {
            state = State.SHOOTING;
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
            case SHOOTING:
                robot.setLauncherVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_VELOCITY);
                robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
                // Only feed once both wheels are up to speed; closes again if a shot drags them down
                if (robot.isLauncherAtVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_TOLERANCE)) {
                    robot.openGate();
                } else {
                    robot.closeGate();
                }
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Launcher L/R", "%.0f / %.0f", robot.getLauncherLeftVelocity(), robot.getLauncherRightVelocity());
        telemetry.addData("At speed", robot.isLauncherAtVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_TOLERANCE));
    }

    @Override
    public void stop() {
        robot.drive(0, 0, 0);
        robot.setIntakePower(0);
        robot.setLauncherVelocity(0, 0);
    }
}
