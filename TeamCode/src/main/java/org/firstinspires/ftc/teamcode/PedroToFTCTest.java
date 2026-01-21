package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;
import org.firstinspires.ftc.teamcode.subsystems.UpdatedLocalizerSubsys;

@TeleOp
public class PedroToFTCTest extends CommandOpMode {
    private Follower follower;
    private UpdatedLocalizerSubsys localizer;
    private TurretSubsys turret;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    public final Pose pedroStartPose = new Pose(72, 72, 0);

    @Override
    public void initialize() {
        RobotConstants.robotTeam = RobotConstants.Team.BLUE;
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(pedroStartPose);
        follower.update();
        localizer = new UpdatedLocalizerSubsys(hardwareMap, follower);
        turret = new TurretSubsys(hardwareMap);
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        localizer.updatePoses();

        Pose pedroPose = follower.getPose();
        Pose2D ftcPose = localizer.getFTCPose();

        turret.turretTrack(ftcPose);

        telemetryData.addData("pedroX", follower.getPose().getX());
        telemetryData.addData("pedroY", follower.getPose().getY());
        telemetryData.addData("pedroHead", Math.toDegrees(follower.getHeading()));
        telemetryData.addData("ftcX", ftcPose.getX(DistanceUnit.INCH));
        telemetryData.addData("ftcY", ftcPose.getX(DistanceUnit.INCH));
        telemetryData.addData("ftcHead", ftcPose.getHeading(AngleUnit.DEGREES));
        telemetryData.update();
    }
}
