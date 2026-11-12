package org.firstinspires.ftc.team5898;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team5898.Hardware.Robot;
import org.firstinspires.ftc.team5898.Hardware.RobotConstants;
import org.firstinspires.ftc.team5898.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Odometry", preselectTeleOp = "A Super Invincible Awesome Explosive TeleOP")

public class AutoOdometry extends OpMode {

    private Follower follower;
    private Robot robot;
    private final ElapsedTime shootTimer = new ElapsedTime();
    // The field is point-symmetric: blue is red rotated 180° around the center
    private static final double FIELD_CENTER = 72;
    private Pose startPose, controlPose, parkPose;

    // Key field positions are written for red; this rotates them for blue
    private static PoseFactory poses(boolean isRed) {
        return isRed ? PoseFactory.degrees() : PoseFactory.degrees().mirrorAroundPoint(FIELD_CENTER, FIELD_CENTER);
    }

    // Public so TeleOp can start here when it has no pose from auto
    public static Pose parkPose(boolean isRed) {
        return poses(isRed).of(9.82, 96.24, 90);
    }

    private Path pathGeneratorLinear(Pose pose1, Pose pose2) {
        return line(pose1, pose2).linear(pose1, pose2);
    }

    private Path pathGeneratorLinearWithControlPoint(Pose pose1, Pose pose2, Pose control) {
        return curve(pose1, control, pose2).linear(pose1, pose2);
    }

    // No ball sensor, so shoot for a fixed time, then shut the launcher off before driving
    private Command shootPreloads() {
        return Command.build()
                .setStart(shootTimer::reset)
                .setExecute(() -> {
                    // Only feed once both wheels are up to speed; stop again if a shot drags them down
                    if (robot.isLauncherAtVelocity(RobotConstants.Auto.LAUNCHER_VELOCITY, RobotConstants.Auto.LAUNCHER_TOLERANCE)) {
                        robot.setIntakePower(RobotConstants.TeleOp.INTAKE_POWER);
                        robot.openGate();
                    } else {
                        robot.setIntakePower(0);
                        robot.closeGate();
                    }
                })
                .setDone(() -> shootTimer.seconds() >= RobotConstants.Auto.SHOOT_TIME_SEC)
                .setEnd(end -> {
                    robot.setIntakePower(0);
                    robot.closeGate();
                    robot.setLauncherVelocity(0, 0);
                });
    }

    @Override
    public void init() {
        Scheduler.reset();
        // Robot first so Pedro's drive motor setup wins
        robot = new Robot(hardwareMap);
        robot.applyLauncherPIDF(RobotConstants.PIDFLauncherAuto.kP, RobotConstants.PIDFLauncherAuto.kI,
                RobotConstants.PIDFLauncherAuto.kD, RobotConstants.PIDFLauncherAuto.kF);
        robot.closeGate();
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void init_loop() {
        // Pick alliance until START
        if (gamepad1.aWasPressed()) {
            Constants.isRed = !Constants.isRed;
        }
        telemetry.addData("Alliance (A)", Constants.isRed ? "RED" : "BLUE");
        telemetry.update();
    }

    @Override
    public void start() {
        // Key field positions, written for red; rotated for blue
        PoseFactory p = poses(Constants.isRed);
        startPose = p.of(58.2, 8.2, 90);
        controlPose = p.of(26.65, 22.58, 90);
        parkPose = parkPose(Constants.isRed);
        follower.setPose(startPose);

        // Spin up right away so the wheels are ready sooner
        robot.setLauncherVelocity(RobotConstants.Auto.LAUNCHER_VELOCITY, RobotConstants.Auto.LAUNCHER_VELOCITY);

        // follow() has no requirements, so separate schedule() calls would all start at once
        schedule(sequential(
                shootPreloads(),
                follow(follower, pathGeneratorLinearWithControlPoint(startPose, parkPose, controlPose))
        ));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // Save every loop so TeleOp gets the latest pose however auto ends
        Constants.lastAutoPose = follower.pose();

        telemetry.addData("Shoot timer", "%.1f", shootTimer.seconds());
        telemetry.addData("Launcher L/R", "%.0f / %.0f", robot.getLauncherLeftVelocity(), robot.getLauncherRightVelocity());
        telemetry.addData("At speed", robot.isLauncherAtVelocity(RobotConstants.Auto.LAUNCHER_VELOCITY, RobotConstants.Auto.LAUNCHER_TOLERANCE));
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
