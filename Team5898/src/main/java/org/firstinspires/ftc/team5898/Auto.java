package org.firstinspires.ftc.team5898;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

// Autonomous for meet 1
// Shoot once, and park, red and blue alliance support
@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    // Key moves (blue side; mirrored for red). Placeholders, measure on the field
    private static final double HIVE_HEADING = 0;
    private static final double TO_GARDEN_FORWARD = -30;
    private static final double TO_GARDEN_STRAFE = 0;
    // Park: strafe + is right on blue, so red goes left 34, forward 75, left 20
    private static final double PARK_SIDE_STRAFE = 34;
    private static final double PARK_FORWARD_INCHES = 75;
    private static final double PARK_IN_STRAFE = 20;

    private boolean isRed = true;
    private boolean shouldPark = true;
    private final ElapsedTime timer = new ElapsedTime();
    // Heading the drives hold, in degrees; 0 is the START facing until turnTo() changes it
    private double holdHeading = 0;
    private Robot robot;
    private TelemetryManager panels;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.closeGate();

        // Pick alliance and parking until START
        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) {
                isRed = !isRed;
            }
            if (gamepad1.bWasPressed()) {
                shouldPark = !shouldPark;
            }
            telemetry.addData("Alliance (A)", isRed ? "RED" : "BLUE");
            telemetry.addData("Park (B)", shouldPark ? "YES" : "NO");
            telemetry.update();
        }
        if (isStopRequested()) return;
        // Zero heading at START so drift or repositioning during init doesn't skew the turns
        robot.resetHeading();
        resetRuntime();

        // Spin up while driving to the hive so shooting doesn't wait for the wheels
        robot.setLauncherVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_VELOCITY);

        // Aim and shoot the preloads
        shootFor(Constants.Auto.SHOOT_TIME_SEC);

        // Refill at the garden only if there is still time to shoot again and park
        if (getRuntime() < Constants.Auto.REFILL_CUTOFF_SEC) {
            robot.setLauncherVelocity(0, 0);
            robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
            driveAndWait(TO_GARDEN_FORWARD, TO_GARDEN_STRAFE);

            // Drive back to the hive, spinning up on the way
            robot.setIntakePower(0);
            robot.setLauncherVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.TeleOp.LAUNCHER_VELOCITY);
            driveAndWait(-TO_GARDEN_FORWARD, -TO_GARDEN_STRAFE);

            // Aim and shoot the refill
            turnTo(HIVE_HEADING);
            shootFor(Constants.Auto.SHOOT_TIME_SEC);
        }

        // Shut the launcher off before parking
        robot.setLauncherVelocity(0, 0);
        robot.setIntakePower(0);
        robot.closeGate();

        // Park: strafe out, drive up the field, strafe in
        if (shouldPark) {
            driveAndWait(0, PARK_SIDE_STRAFE);
            driveAndWait(-10, 0);
            driveAndWait(PARK_FORWARD_INCHES, 0);
            driveAndWait(0, PARK_IN_STRAFE);
        }
    }

    // No ball sensor, so shoot for a fixed time
    private void shootFor(double seconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            // Only feed once both wheels are up to speed; stop again if a shot drags them down
            if (robot.isLauncherAtVelocity(Constants.TeleOp.LAUNCHER_VELOCITY, Constants.Auto.LAUNCHER_TOLERANCE)) {
                robot.setIntakePower(Constants.TeleOp.INTAKE_POWER);
                robot.openGate();
            } else {
                robot.setIntakePower(0);
                robot.closeGate();
            }
            telemetry.addData("Time", "%.1f", getRuntime());
            telemetry.addData("Launcher L/R", "%.0f / %.0f", robot.getLauncherLeftVelocity(), robot.getLauncherRightVelocity());
            telemetry.update();
        }
        robot.setIntakePower(0);
        robot.closeGate();
    }

    private void driveAndWait(double forwardInches, double strafeInches) {
        robot.startDriveInches(forwardInches, isRed ? -strafeInches : strafeInches, Constants.Auto.DRIVE_POWER);
        timer.reset();
        // Timeout so a robot stuck on a wall doesn't burn the rest of auto
        while (opModeIsActive() && robot.updateDrive(Math.toRadians(holdHeading)) && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            showHeading();
        }
        robot.stopDrive();
    }

    private void turnTo(double headingDeg) {
        double target = isRed ? -headingDeg : headingDeg;
        holdHeading = target;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            double error = AngleUnit.normalizeDegrees(target - Math.toDegrees(robot.getHeadingRad()));
            if (Math.abs(error) < Constants.Auto.TURN_TOLERANCE) {
                break;
            }
            // IMU yaw is counterclockwise-positive but drive() turns clockwise, so flip the sign
            double turn = Range.clip(-Constants.Auto.TURN_KP * error, -Constants.Auto.TURN_MAX_POWER, Constants.Auto.TURN_MAX_POWER);
            // Small errors give too little power to move, so the turn would stall until the timeout
            turn = Math.copySign(Math.max(Math.abs(turn), Constants.Auto.TURN_MIN_POWER), turn);
            robot.drive(0, 0, turn);
            showHeading();
        }
        robot.drive(0, 0, 0);
    }

    // Graph heading against its target in Panels to tune TURN_KP
    private void showHeading() {
        panels.addData("heading", Math.toDegrees(robot.getHeadingRad()));
        panels.addData("targetHeading", holdHeading);
        panels.update(telemetry);
    }
}
