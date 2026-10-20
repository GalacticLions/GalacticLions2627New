/**
 * @Author: Eli Xiao
 */
package org.firstinspires.ftc.team5898.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain and IMU. Create one in an OpMode's init and use it to drive.
 */
public class Robot {
    private final DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    private final DcMotor[] driveMotors;
    private final DcMotorEx launcherRight, launcherLeft;
    private final IMU imu;
    private final Servo gate;

    /**
     * Gets the drive motors and IMU from the hardware map and sets them up.
     *
     * @param hardwareMap the OpMode's hardware map
     */
    public Robot(HardwareMap hardwareMap) {
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
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
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
        double rotatedForward = forward * Math.cos(heading) + strafe * Math.sin(heading);
        double rotatedStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);
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
        int forward = (int) Math.round(forwardInches * Constants.Drive.COUNTS_PER_INCH);
        int strafe = (int) Math.round(strafeInches * Constants.Drive.COUNTS_PER_INCH * Constants.Auto.STRAFE_MULTIPLIER);
        // Same wheel mixing as drive(); targets must be set before switching to RUN_TO_POSITION
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + forward + strafe);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + forward - strafe);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + forward - strafe);
        backRight.setTargetPosition(backRight.getCurrentPosition() + forward + strafe);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(power));
        }
    }

    public boolean isDriveBusy() {
        for (DcMotor motor : driveMotors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public void stopDrive() {
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void applyLauncherPIDF() {
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                Constants.PIDFLauncherLeft.kP, Constants.PIDFLauncherLeft.kI,
                Constants.PIDFLauncherLeft.kD, Constants.PIDFLauncherLeft.kF));
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                Constants.PIDFLauncherRight.kP, Constants.PIDFLauncherRight.kI,
                Constants.PIDFLauncherRight.kD, Constants.PIDFLauncherRight.kF));
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

    public boolean isLauncherAtVelocity(double targetVelocity) {
        double tolerance = Constants.TeleOp.LAUNCHER_TOLERANCE;
        return Math.abs(launcherLeft.getVelocity() - targetVelocity) < tolerance
                && Math.abs(launcherRight.getVelocity() - targetVelocity) < tolerance;
    }

    public void closeGate() {
        gate.setPosition(Constants.Gate.GATE_CLOSED);
    }

    public void openGate() {
        gate.setPosition(Constants.Gate.GATE_OPEN);
    }
}
