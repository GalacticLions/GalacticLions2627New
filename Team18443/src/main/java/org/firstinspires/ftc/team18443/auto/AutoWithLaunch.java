package org.firstinspires.ftc.team18443.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AutoWithLaunch", group = "StarterBot")
public class AutoWithLaunch extends OpMode {
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    final double DRIVE_SPEED = 0.5;
    final double STRAFE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    final double TRACK_WIDTH_MM = 370;
    final double WHEELBASE_MM = 380;
    final double STRAFE_CORRECTION = 1.15;
    final double TOLERANCE_MM = 10;

    double intakePower = 0;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private enum AutonomousState {
        DRIVE,
        ROTATE,
        COMPLETE,
        DRIVEB
    }

    private AutonomousState autonomousState;

    @Override
    public void init() {
        autonomousState = AutonomousState.DRIVE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        resetDriveEncoders();

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    @Override
    public void loop() {
        intakePower = 0;

        switch (autonomousState) {
            case DRIVE:
                if (drive(DRIVE_SPEED, 380, DistanceUnit.MM, 2)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.ROTATE;
                }
                break;

            case ROTATE:
                if (rotate(ROTATE_SPEED, 90, AngleUnit.DEGREES, 2)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.DRIVEB;
                }
                // fall through
            case DRIVEB:
                if (drive(DRIVE_SPEED, -1000, DistanceUnit.MM, 2)) {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.COMPLETE;
                }
                // fall through
            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Current Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.addData("Target Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition(),
                leftBackDrive.getTargetPosition(), rightBackDrive.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    void launch(boolean input) {
    }

    void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean runToTargets(double speed, int lf, int rf, int lb, int rb, double holdSeconds) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        double maxError = Math.max(
                Math.max(Math.abs(lf - leftFrontDrive.getCurrentPosition()), Math.abs(rf - rightFrontDrive.getCurrentPosition())),
                Math.max(Math.abs(lb - leftBackDrive.getCurrentPosition()), Math.abs(rb - rightBackDrive.getCurrentPosition())));

        if (maxError > TOLERANCE_MM * TICKS_PER_MM) {
            driveTimer.reset();
        }
        return driveTimer.seconds() > holdSeconds;
    }

    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        int target = (int) (distanceUnit.toMm(distance) * TICKS_PER_MM);
        return runToTargets(speed, target, target, target, target, holdSeconds);
    }

    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        int target = (int) (distanceUnit.toMm(distance) * STRAFE_CORRECTION * TICKS_PER_MM);
        return runToTargets(speed, target, -target, -target, target, holdSeconds);
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        double turnRadiusMm = Math.hypot(TRACK_WIDTH_MM / 2, WHEELBASE_MM / 2);
        double targetMm = angleUnit.toRadians(angle) * turnRadiusMm;
        int rightTarget = (int) (targetMm * TICKS_PER_MM);
        int leftTarget = -rightTarget;
        return runToTargets(speed, leftTarget, rightTarget, leftTarget, rightTarget, holdSeconds);
    }
}
