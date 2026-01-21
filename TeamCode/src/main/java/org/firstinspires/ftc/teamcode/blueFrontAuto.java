package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsys;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;
import org.firstinspires.ftc.teamcode.subsystems.UpdatedLocalizerSubsys;

@Autonomous
public class blueFrontAuto extends CommandOpMode {
    private Follower follower;
    private TelemetryData telemetryData = new TelemetryData(telemetry);
    private UpdatedLocalizerSubsys updatedLocalizer;

    private FlywheelSubsys flywheel;
    private HoodSubsys hood;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    private IntakeSubsys intake;
    public double distanceToGoal = 0;
    public boolean shooterOn = false;

    public final Pose startPose = RobotConstants.blueFrontStart;
    public final Pose shootPose = new Pose(36, 111, Math.toRadians(200));

    private PathChain shootFirst;

    public void buildPaths(){
        shootFirst = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    private RunCommand updatePoses = new RunCommand(() -> updatedLocalizer.updatePoses());
    private RunCommand updateDistance = new RunCommand(()-> distanceToGoal = updatedLocalizer.getDistance());
    private RunCommand trackTurret = new RunCommand(() -> turret.turretTrack(updatedLocalizer.getFTCPose()));
    private RunCommand runHood = new RunCommand(()-> hood.runHoodRegression(distanceToGoal));
    private InstantCommand runIntake = new InstantCommand(()-> intake.runIntake(1));
    private InstantCommand stopIntake = new InstantCommand(()-> intake.runIntake(0));
    private InstantCommand toggleShooter = new InstantCommand(()-> shooterOn = !shooterOn);

    /*
            new WaitCommand(750), runIntake, new WaitCommand(750), stopIntake,
            new WaitCommand(100), runIntake, new WaitCommand(500), stopIntake, toggleShooter
     */ //shootSequence

    private RunCommand runShooter = new RunCommand(()->{
        if (shooterOn){
            flywheel.runFlywheelRegression(distanceToGoal);
            stopper.stopperOpen();
        } else {
            flywheel.idleFlywheel();
            stopper.stopperClose();
        }
    });

    @Override
    public void initialize() {
        super.reset();
        RobotConstants.robotTeam = RobotConstants.Team.BLUE;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        updatedLocalizer = new UpdatedLocalizerSubsys(hardwareMap, follower);
        updatedLocalizer.updatePoses();

        flywheel = new FlywheelSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        stopper = new StopperSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
        intake = new IntakeSubsys(hardwareMap);
        stopper.stopperClose();

        buildPaths();

        schedule(
                new RunCommand(()-> follower.update()), updatePoses, updateDistance,
                trackTurret, runHood, runShooter,
                new SequentialCommandGroup(
                        toggleShooter,
                        new FollowPathCommand(follower, shootFirst, true),
                        new WaitCommand(750), runIntake, new WaitCommand(2000),
                        stopIntake,toggleShooter
                )
        );
    }

    @Override
    public void run(){
        super.run();

        telemetryData.addData("ftcX", updatedLocalizer.getFTCPose().getX(DistanceUnit.INCH));
        telemetryData.addData("ftcY", updatedLocalizer.getFTCPose().getY(DistanceUnit.INCH));
        telemetryData.addData("distanceReported", updatedLocalizer.getDistance());
        telemetryData.addData("actual distance", distanceToGoal);
        telemetryData.update();
    }
}
