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
    private final ElapsedTime settleTimer = new ElapsedTime();
    // Heading the drives hold, in degrees; 0 is the START facing until turnTo() changes it
    private double holdHeading = 0;
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
        while (opModeIsActive() && robot.updateDrive(Math.toRadians(holdHeading)) && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            idle();
        }
        robot.stopDrive();
    }

    // Same as above, but turns to headingDeg on the way and holds it for later moves. turnTo() finishes
    // the turn in case the move was too short to get there
    private void driveAndWait(double forwardInches, double strafeInches, double headingDeg) {
        holdHeading = isRed ? -headingDeg : headingDeg;
        driveAndWait(forwardInches, strafeInches);
        turnTo(headingDeg);
    }

    private void turnTo(double headingDeg) {
        double target = isRed ? -headingDeg : headingDeg;
        holdHeading = target;
        timer.reset();
        settleTimer.reset();
        while (opModeIsActive() && timer.seconds() < Constants.Auto.STEP_TIMEOUT_SEC) {
            double error = AngleUnit.normalizeDegrees(target - Math.toDegrees(robot.getHeadingRad()));
            if (Math.abs(error) < Constants.Auto.TURN_TOLERANCE) {
                // Brake and wait inside tolerance: momentum can carry it back out, and min power here would chatter
                robot.drive(0, 0, 0);
                if (settleTimer.seconds() > Constants.Auto.TURN_SETTLE_SEC) {
                    break;
                }
            } else {
                settleTimer.reset();
                // IMU yaw is counterclockwise-positive but drive() turns clockwise, so flip the sign
                double turn = Range.clip(-Constants.Auto.TURN_KP * error, -Constants.Auto.TURN_MAX_POWER, Constants.Auto.TURN_MAX_POWER);
                // Small errors give too little power to move, so the turn would stall until the timeout
                turn = Math.copySign(Math.max(Math.abs(turn), Constants.Auto.TURN_MIN_POWER), turn);
                robot.drive(0, 0, turn);
            }
        }
        robot.drive(0, 0, 0);
    }
}
