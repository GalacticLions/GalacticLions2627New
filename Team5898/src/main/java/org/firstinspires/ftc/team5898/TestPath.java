package org.firstinspires.ftc.team5898;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.team5898.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Test Path")
public class TestPath extends OpMode {

    private Follower follower;
    private static final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(58.5, 8.4, 90);
    private final Pose firstPose = p.of(33.09, 11.9, 90);
    private final Pose secondPose = p.of(32.85, 112.71, 0);
    private final Pose secondControlPose = p.of(13.07, 79.48, 45);
    public static final Pose finalPose = p.of(118.68, 49.7, 270);
    private final Pose finalControlPose = p.of(138.37, 141.5, 45);

    private Path pathGeneratorLinear(Pose pose1, Pose pose2) {
        return line(pose1, pose2).linear(pose1, pose2);
    }

    private Path pathGeneratorLinearWithControlPoint(Pose pose1, Pose pose2, Pose control) {
        return curve(pose1, control, pose2).linear(pose1, pose2);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        // follow() has no requirements, so separate schedule() calls would all start at once
        schedule(sequential(
                follow(follower, pathGeneratorLinear(startPose, firstPose)),
                follow(follower, pathGeneratorLinearWithControlPoint(firstPose, secondPose, secondControlPose)),
                follow(follower, pathGeneratorLinearWithControlPoint(secondPose, finalPose, finalControlPose))
        ));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // Save every loop so TeleOp gets the latest pose however auto ends
        Constants.lastAutoPose = follower.pose();
    }

    @Override
    public void stop() {

    }
}
