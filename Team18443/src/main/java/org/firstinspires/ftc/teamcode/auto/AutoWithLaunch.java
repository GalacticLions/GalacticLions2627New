/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the followingv disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This file includes an autonomous file for the goBILDA® StarterBot with Mecanum Wheels for the
 * 2026-2027 FIRST® Tech Challenge season BIOBUZZ™. It uses a mecanum wheel drivetrain, one motor
 * driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * This robot starts up against the playing field wall and launches all four projectiles
 * before driving away from the wall.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 *
 * MECANUM NOTES:
 * Each of the four wheels is driven to its own encoder target using RUN_TO_POSITION.
 *   - Driving forward/back: all four wheels turn the same direction.
 *   - Strafing: diagonal pairs turn together (LF + RB one way, RF + LB the other way).
 *   - Rotating: left side and right side turn opposite directions.
 * These are the same sign patterns used in the mecanumDrive() function of the teleop.
 */

@Autonomous(name="AutoWithLaunch", group="StarterBot")
//@Disabled
public class AutoWithLaunch extends OpMode
{
    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED, STRAFE_SPEED and
     * ROTATE_SPEED are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific
     * to the motor ratio that we use in the kit; if you're using a different motor, this value can
     * be found on the product page for the motor you're using.
     *
     * TRACK_WIDTH_MM is the distance between the centers of the left and right wheels, and
     * WHEELBASE_MM is the distance between the centers of the front and back wheels.
     * TODO: These two values are used to calculate rotation. They were NOT carried over from the
     * tank drive (the mecanum chassis is a different size), so MEASURE YOUR ROBOT and update them.
     *
     * STRAFE_CORRECTION compensates for the fact that mecanum wheels lose some efficiency when
     * moving sideways, so the robot travels less than expected for the same wheel rotation.
     * TODO: Tune this value. Command a 1000 mm strafe, measure the actual distance, and set
     * STRAFE_CORRECTION = 1000 / actual. Many robots land between 1.1 and 1.2.
     */
    final double DRIVE_SPEED = 0.5;
    final double STRAFE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 370;   // TODO: measure on the mecanum chassis
    final double WHEELBASE_MM = 380;     // TODO: measure on the mecanum chassis
    final double STRAFE_CORRECTION = 1.15; // TODO: tune on the field


    // Create a variable to set to the intake.
    double intakePower = 0;

    /*
     * Here we create a timer which we use in our drive function, and a timer to control the flow
     * of our auto as a whole.
     */
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "AutonomousState." It reflects the current state of our auto.
     * It starts at LAUNCH, and we can use higher level code to cycle through these states.
     * This allows us to write functions and autonomous routines in a way that avoids
     * loops within loops, and "waits."
     */
    private enum AutonomousState {
        DRIVE,
        ROTATE,
        COMPLETE,
        DRIVEB,
        LAUNCH
    }

    /*
     * Here we create the instance of AutonomousState that we use in code. This creates a unique
     * object which can store the current condition of the shooter. In other applications,
     * you may have multiple copies of the same enum which have different names.
     * Here we just have one.
     */
    private AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting
         * autonomousState = AutonomousState.LAUNCH. Later in our code, we will progress through
         * the state machine by moving to other enum members.
         */
        autonomousState = AutonomousState.DRIVE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         * These names match the mecanum teleop (note the servo is "windmillServo" here, while
         * the tank drive code used "windmill").
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
//        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
//        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * Motor directions are copied from the mecanum teleop, so that a positive target position
         * on every wheel drives the robot forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        resetDriveEncoders();

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        //intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        //launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
//        leftIntakeServo.setPower(0);
//        rightIntakeServo.setPower(0);
//        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
//        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
//        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        autoTimer.reset();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        /*
         * launch() adds to intakePower while feeding, so we start each loop from zero. This keeps
         * the value from growing without bound over the 10 second launch.
         */
        intakePower = 0;

        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState) {
            case DRIVE:
                if(drive(DRIVE_SPEED, 380, DistanceUnit.MM, 2)){
                    resetDriveEncoders();
                    autonomousState = AutonomousState.ROTATE;
                }
                break;
            case ROTATE:
                if(rotate(ROTATE_SPEED, 90, AngleUnit.DEGREES, 2))
                {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.DRIVEB;
                }
            case DRIVEB:
                if(drive(DRIVE_SPEED, -1000, DistanceUnit.MM, 2))
                {
                    resetDriveEncoders();
                    autonomousState = AutonomousState.COMPLETE;
                }

            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        //intake.setPower(intakePower);

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Current Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.addData("Target Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition(),
                leftBackDrive.getTargetPosition(), rightBackDrive.getTargetPosition());
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    void launch(boolean input) {
        if (input) {
            //launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            //launcher.setVelocity(0);
        }

//        if (input && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
//            //windmillServo.setPower(1);
//            intakePower += 0.5;
//        } else {
//            //windmillServo.setPower(0);
//        }
    }

    /**
     * Stops the drive motors and zeroes their encoders. Call this between drive/strafe/rotate
     * steps, because the target positions are measured from the last reset.
     */
    void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Shared helper that sends each wheel to its own target (in encoder ticks) and reports
     * whether all four wheels have been within tolerance for long enough.
     *
     * @param speed From 0-1
     * @param lf target ticks for the left front wheel
     * @param rf target ticks for the right front wheel
     * @param lb target ticks for the left back wheel
     * @param rb target ticks for the right back wheel
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return true if ALL wheels are within tolerance of their targets for more than holdSeconds.
     */
    private boolean runToTargets(double speed, int lf, int rf, int lb, int rb, double holdSeconds) {
        final double TOLERANCE_MM = 10;

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

        /*
         * With four wheels, we look at the WORST error of any wheel. We only consider the robot
         * "at position" when every wheel is within tolerance. If any wheel is still outside of
         * tolerance, we reset the driveTimer, so it can only count past holdSeconds after all
         * wheels have arrived.
         */
        double maxError = Math.max(
                Math.max(Math.abs(lf - leftFrontDrive.getCurrentPosition()),
                        Math.abs(rf - rightFrontDrive.getCurrentPosition())),
                Math.max(Math.abs(lb - leftBackDrive.getCurrentPosition()),
                        Math.abs(rb - rightBackDrive.getCurrentPosition())));

        if (maxError > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Drives straight forward (positive) or backward (negative).
     *
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        int target = (int) (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // Driving straight: every wheel turns the same direction.
        return runToTargets(speed, target, target, target, target, holdSeconds);
    }

    /**
     * Strafes sideways. Positive distance moves RIGHT, negative moves LEFT.
     *
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if all wheels are within tolerance of the target for more than holdSeconds.
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        /*
         * Mecanum wheels slide less efficiently sideways, so we multiply the requested distance
         * by STRAFE_CORRECTION to make the robot actually travel the distance we asked for.
         */
        int target = (int) (distanceUnit.toMm(distance) * STRAFE_CORRECTION * TICKS_PER_MM);

        /*
         * To strafe right, the front-left and back-right wheels turn forward while the
         * front-right and back-left wheels turn backward (same pattern as the teleop).
         */
        return runToTargets(speed, target, -target, -target, target, holdSeconds);
    }

    /**
     * Rotates in place. Positive angle rotates counterclockwise (same as the tank drive code).
     *
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if all wheels are within tolerance of the target for more than holdSeconds.
     *         False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        /*
         * On a tank drive, each wheel travels in a circle with a radius of half the track width.
         * On a four-wheel mecanum drive the wheels sit at the corners of a rectangle, so the
         * radius is the distance from the center of the robot to a wheel, which is half of the
         * diagonal between the wheels. As before, we use radians so that distance traveled
         * is simply angle * radius.
         */
        double turnRadiusMm = Math.hypot(TRACK_WIDTH_MM / 2.0, WHEELBASE_MM / 2.0);
        double targetMm = angleUnit.toRadians(angle) * turnRadiusMm;

        int rightTarget = (int) (targetMm * TICKS_PER_MM);
        int leftTarget = -rightTarget;

        return runToTargets(speed, leftTarget, rightTarget, leftTarget, rightTarget, holdSeconds);
    }
}