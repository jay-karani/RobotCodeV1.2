package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class TestPath extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses
    public final Pose startPose = new Pose(55.5, 9, Math.toRadians(90));
    public final Pose pickupFirst = new Pose(40, 36, Math.toRadians(180));

    // Pathchains
    private PathChain goToFirst;

    public void buildPaths(){
        goToFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupFirst))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupFirst.getHeading())
                .setBrakingStart(0)
                .setBrakingStart(0)
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        schedule(new FollowPathCommand(follower, goToFirst, true).setGlobalMaxPower(1));
    }

    @Override
    public void run(){
        super.run();

        follower.update();
        follower.getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
    }
}
