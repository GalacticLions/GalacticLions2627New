package org.firstinspires.ftc.team5898.Template;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team5898.Hardware.Constants;
import org.firstinspires.ftc.team5898.Hardware.Robot;

// A template for non-state-machine autonomous
@Disabled
public class AutoTemplate extends LinearOpMode {
    // Key moves (blue side; mirrored for red)
    private static final double TO_GOAL_INCHES = 24;
    private static final double GOAL_HEADING = 45;
    private static final double PARK_STRAFE_INCHES = 18;

    private final boolean isRed = false;
    private final ElapsedTime timer = new ElapsedTime();
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.resetHeading();
        waitForStart();

        // Drive forward to the goal
        driveAndWait(TO_GOAL_INCHES, 0);

        // Turn to face the goal
        turnTo(GOAL_HEADING);

        // Strafe out of the way to park
        driveAndWait(0, PARK_STRAFE_INCHES);
    }

    private void driveAndWait(double forwardInches, double strafeInches) {
        robot.startDriveInches(forwardInches, isRed ? -strafeInches : strafeInches, Constants.Auto.DRIVE_POWER);
        timer.reset();
        // Timeout so a robot stuck on a wall doesn't burn the rest of auto
        while (opModeIsActive() && robot.isDriveBusy() && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            idle();
        }
        robot.stopDrive();
    }

    private void turnTo(double headingDeg) {
        double target = isRed ? -headingDeg : headingDeg;
        timer.reset();
        while (opModeIsActive() && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            double error = AngleUnit.normalizeDegrees(target - Math.toDegrees(robot.getHeadingRad()));
            if (Math.abs(error) < Constants.Auto.TURN_TOLERANCE) {
                break;
            }
            // IMU yaw is counterclockwise-positive but drive() turns clockwise, so flip the sign
            double turn = Range.clip(-Constants.Auto.TURN_KP * error, -Constants.Auto.TURN_MAX_POWER, Constants.Auto.TURN_MAX_POWER);
            robot.drive(0, 0, turn);
        }
        robot.drive(0, 0, 0);
    }
}
