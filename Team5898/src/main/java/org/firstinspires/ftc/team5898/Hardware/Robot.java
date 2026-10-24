/**
 * @Author: Eli Xiao
 */
package org.firstinspires.ftc.team5898.Hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain and IMU. Create one in an OpMode's init and use it to drive.
 */
public class Robot {
    private final DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private final DcMotor[] driveMotors;
    private final int[] driveStart = new int[4];
    private double driveTargetForward, driveTargetStrafe, drivePower;
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final DcMotorEx launcherRight, launcherLeft;
    private final IMU imu;
    private final Servo gate;

    /**
     * Gets the drive motors and IMU from the hardware map and sets them up.
     *
     * @param hardwareMap the OpMode's hardware map
     */
    public Robot(HardwareMap hardwareMap) {
        // Read all of a hub's encoders in one call instead of one call per motor, so loops run faster
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft = hardwareMap.get(DcMotor.class, Constants.Hardware.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, Constants.Hardware.MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, Constants.Hardware.MOTOR_BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, Constants.Hardware.MOTOR_BACK_RIGHT);
        launcherRight = hardwareMap.get(DcMotorEx.class, Constants.Hardware.MOTOR_LAUNCHER_RIGHT);
        launcherLeft = hardwareMap.get(DcMotorEx.class, Constants.Hardware.MOTOR_LAUNCHER_LEFT);
        intake = hardwareMap.get(DcMotor.class, Constants.Hardware.MOTOR_INTAKE);
        gate = hardwareMap.get(Servo.class, Constants.Hardware.SERVO_GATE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        for (DcMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Motor modes persist between OpModes, so undo any RUN_TO_POSITION left over from auto
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for (DcMotor launcher : new DcMotor[]{launcherRight, launcherLeft}) {
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        applyLauncherPIDF();

        imu = hardwareMap.get(IMU.class, Constants.Hardware.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    /**
     * Drives the robot relative to itself. All inputs are clipped together so no motor goes over full power.
     *
     * @param forward power to drive forward, from -1 to 1
     * @param strafe power to strafe right, from -1 to 1
     * @param turn power to turn clockwise, from -1 to 1
     */
    public void drive(double forward, double strafe, double turn) {
        double denominator = Math.max(1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        frontRight.setPower((forward - strafe - turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }

    /**
     * Drives the robot relative to the field, using the IMU heading.
     *
     * @param forward power to drive away from the driver, from -1 to 1
     * @param strafe power to strafe right, from -1 to 1
     * @param turn power to turn clockwise, from -1 to 1
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeadingRad();
        // Yaw is counterclockwise-positive, so rotate the stick by -heading into the robot's frame
        double rotatedForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double rotatedStrafe = strafe * Math.cos(heading) + forward * Math.sin(heading);
        drive(rotatedForward, rotatedStrafe, turn);
    }

    /**
     * Reads the robot's heading from the IMU.
     *
     * @return heading in radians
     */
    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Sets the current facing as heading zero.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    public void startDriveInches(double forwardInches, double strafeInches, double power) {
        driveTargetForward = forwardInches * Constants.Drive.COUNTS_PER_INCH;
        driveTargetStrafe = strafeInches * Constants.Drive.COUNTS_PER_INCH * Constants.Auto.STRAFE_MULTIPLIER;
        drivePower = Math.abs(power);
        driveTimer.reset();
        for (int i = 0; i < driveMotors.length; i++) {
            driveStart[i] = driveMotors[i].getCurrentPosition();
            // Velocity control so a weak or loaded motor keeps pace with the others
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // TODO docstring
    // Wheels slip (strafing especially), so encoder targets alone let the robot curve; the IMU corrects that
    public boolean updateDrive(double headingRad) {
        double fl = frontLeft.getCurrentPosition() - driveStart[0];
        double fr = frontRight.getCurrentPosition() - driveStart[1];
        double bl = backLeft.getCurrentPosition() - driveStart[2];
        double br = backRight.getCurrentPosition() - driveStart[3];
        // Inverse of the drive() mixing; turning cancels out of both sums
        double forwardError = driveTargetForward - (fl + fr + bl + br) / 4;
        double strafeError = driveTargetStrafe - (fl - bl - fr + br) / 4;
        double distance = Math.hypot(forwardError, strafeError);
        if (distance / Constants.Drive.COUNTS_PER_INCH < Constants.Auto.DRIVE_TOLERANCE_INCHES) {
            drive(0, 0, 0);
            return false;
        }
        // Ramp up so the wheels don't slip at the start (slip fools the encoders), and brake on a sqrt
        // curve (constant deceleration) so we stop on target without crawling the last few inches
        double accelLimit = driveTimer.seconds() / Constants.Auto.DRIVE_ACCEL_SEC;
        double decelLimit = Math.sqrt(distance / Constants.Drive.COUNTS_PER_INCH / Constants.Auto.DRIVE_DECEL_INCHES);
        double speed = Range.clip(Math.min(accelLimit, decelLimit), Constants.Auto.DRIVE_MIN_POWER, drivePower);
        double headingError = AngleUnit.normalizeDegrees(Math.toDegrees(headingRad - getHeadingRad()));
        // IMU yaw is counterclockwise-positive but drive() turns clockwise, so flip the sign
        double turn = Range.clip(-Constants.Auto.TURN_KP * headingError,
                -Constants.Auto.TURN_MAX_POWER, Constants.Auto.TURN_MAX_POWER);
        drive(speed * forwardError / distance, speed * strafeError / distance, turn);
        return true;
    }

    public void stopDrive() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void applyLauncherPIDF() {
        applyPIDF(launcherLeft, Constants.PIDFLauncherLeft.kP, Constants.PIDFLauncherLeft.kI,
                Constants.PIDFLauncherLeft.kD, Constants.PIDFLauncherLeft.kF);
        applyPIDF(launcherRight, Constants.PIDFLauncherRight.kP, Constants.PIDFLauncherRight.kI,
                Constants.PIDFLauncherRight.kD, Constants.PIDFLauncherRight.kF);
    }

    // All-zero gains mean "not tuned yet": keep the motor's factory defaults instead of disabling its velocity loop
    private static void applyPIDF(DcMotorEx motor, double p, double i, double d, double f) {
        if (p == 0 && i == 0 && d == 0 && f == 0) return;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    public void setIntakePower(double intakePower) {
        intake.setPower(intakePower);
    }

    public void setLauncherVelocity(double leftVelocity, double rightVelocity) {
        launcherRight.setVelocity(rightVelocity);
        launcherLeft.setVelocity(leftVelocity);
    }

    public double getLauncherLeftVelocity() {
        return launcherLeft.getVelocity();
    }

    public double getLauncherRightVelocity() {
        return launcherRight.getVelocity();
    }

    public boolean isLauncherAtVelocity(double targetVelocity, double tolerance) {
        return Math.abs(launcherLeft.getVelocity() - targetVelocity) < tolerance
                && Math.abs(launcherRight.getVelocity() - targetVelocity) < tolerance;
    }

    public void closeGate() {
        gate.setPosition(Constants.Gate.GATE_CLOSED);
    }

    public void openGate() {
        gate.setPosition(Constants.Gate.GATE_OPEN);
    }

    public void setGatePosition(double position) {
        gate.setPosition(position);
    }

    public double getGatePosition() {
        return gate.getPosition();
    }
}
