package org.firstinspires.ftc.team5898;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team5898.Hardware.Robot;
import org.firstinspires.ftc.team5898.Hardware.RobotConstants;
import org.firstinspires.ftc.team5898.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@TeleOp(name = "A Super Invincible Awesome Explosive TeleOP")
public class OdometryTeleOP extends OpMode {
    private enum State {IDLE, INTAKING, OUTTAKING, SHOOTING}

    private Follower follower;
    private Robot robot;
    private State state = State.IDLE;
    private final ElapsedTime gateTimer = new ElapsedTime();
    private Pose autoPose;
    private boolean goingToTarget = false;

    @Override
    public void init() {
        // Robot first so Pedro's drive motor setup wins
        robot = new Robot(hardwareMap);
        follower = Constants.create(hardwareMap);
        autoPose = Constants.lastAutoPose;
        follower.setPose(startPose());
    }

    @Override
    public void init_loop() {
        // Pick alliance until START; the park pose moves with it
        if (gamepad1.aWasPressed()) {
            Constants.isRed = !Constants.isRed;
            follower.setPose(startPose());
        }
        telemetry.addData("Alliance (A)", Constants.isRed ? "RED" : "BLUE");
        telemetry.addData("Start pose", (autoPose != null ? "from auto " : "park pose ") + startPose());
    }

    @Override
    public void start() {
        // aWasPressed() remembers presses from init, and those shouldn't drive the robot at START
        gamepad1.aWasPressed();
        // The robot gets moved by hand after this run, so the next TeleOp must not reuse this pose
        Constants.lastAutoPose = null;
    }

    @Override
    public void loop() {
        follower.update();

        // Read A every loop so a press made while steering can't fire later
        boolean aPressed = gamepad1.aWasPressed();
        double forward = -gamepad1.left_stick_y;
        double strafeLeft = -gamepad1.left_stick_x;
        double turnLeft = -gamepad1.right_stick_x;

        // Any stick takes control back from the path; the robot holds the target until then
        if (Math.abs(forward) > RobotConstants.TeleOp.TURN_DEADBAND
                || Math.abs(strafeLeft) > RobotConstants.TeleOp.TURN_DEADBAND
                || Math.abs(turnLeft) > RobotConstants.TeleOp.TURN_DEADBAND) {
            goingToTarget = false;
        } else if (aPressed) {
            goToTarget(Constants.isRed);
        }

        // Drive relative to the robot unless a path is in control
        if (!goingToTarget) {
            follower.manual(forward, strafeLeft, turnLeft);
        }

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
                robot.setIntakePower(RobotConstants.TeleOp.INTAKE_POWER);
                robot.setLauncherVelocity(0, 0);
                robot.closeGate();
                break;
            case OUTTAKING:
                robot.setIntakePower(-RobotConstants.TeleOp.INTAKE_POWER);
                robot.setLauncherVelocity(0, 0);
                robot.closeGate();
                break;
            case SHOOTING:
                robot.setLauncherVelocity(RobotConstants.TeleOp.LAUNCHER_VELOCITY, RobotConstants.TeleOp.LAUNCHER_VELOCITY);
                robot.openGate();
                // Wait before feeding so the gate opens and the wheels spin up; a timer instead of sleep() so driving doesn't freeze
                if (gateTimer.seconds() >= RobotConstants.TeleOp.GATE_OPEN_DELAY_SEC) {
                    robot.setIntakePower(RobotConstants.TeleOp.INTAKE_POWER);
                } else {
                    robot.setIntakePower(0);
                }
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Launcher L/R", "%.0f / %.0f", robot.getLauncherLeftVelocity(), robot.getLauncherRightVelocity());
        telemetry.addData("At speed", robot.isLauncherAtVelocity(RobotConstants.TeleOp.LAUNCHER_VELOCITY, RobotConstants.TeleOp.LAUNCHER_TOLERANCE));
        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Mode", follower.mode());
    }

    @Override
    public void stop() {
        follower.drivetrain.stop();
        robot.setIntakePower(0);
        robot.setLauncherVelocity(0, 0);
    }

    // Continue from where auto stopped; without auto, assume the robot was placed at auto's park pose
    private Pose startPose() {
        return autoPose != null ? autoPose : AutoOdometry.parkPose(Constants.isRed);
    }

    private void goToTarget(boolean isRedAlliance) {
        Pose current = follower.pose();
        // The middle band has obstacles between the robot and the targets, so only drive from above or below it
        if (current.y() >= RobotConstants.Target.AUTO_DRIVE_BELOW_Y
                && current.y() <= RobotConstants.Target.AUTO_DRIVE_ABOVE_Y) return;
        // Upper half of the field goes to the left target, lower half to the right one
        boolean upperHalf = current.y() > RobotConstants.Target.FIELD_MID_Y;
        Pose target = isRedAlliance
                ? (upperHalf ? RobotConstants.Target.RED_LEFT : RobotConstants.Target.RED_RIGHT)
                : (upperHalf ? RobotConstants.Target.BLUE_LEFT : RobotConstants.Target.BLUE_RIGHT);
        // line() throws if the robot is already on the target, so just hold there
        if (Math.hypot(target.x() - current.x(), target.y() - current.y()) < RobotConstants.TeleOp.TARGET_MIN_PATH_INCHES) {
            follower.hold(target);
        } else {
            follower.follow(line(current, target).linear(current, target));
        }
        goingToTarget = true;
    }
}
