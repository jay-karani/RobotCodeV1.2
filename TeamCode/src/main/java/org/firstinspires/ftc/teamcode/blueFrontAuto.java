package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    public final Pose shootPose = new Pose(43, 104, Math.toRadians(180));
    public final Pose firstLine = new Pose(48, 84, Math.toRadians(160));
    public final Pose intakeFirst = new Pose(16, 84, Math.toRadians(160));
    public final Pose cheefFirstWall = new Pose(14,88, Math.toRadians(180));
    public final Pose secondLine = new Pose(48, 64.5, Math.toRadians(200));
    public final Pose intakeSecond = new Pose(9, 64.5, Math.toRadians(200));
    public final Pose secondTwist = new Pose (9, 58, Math.toRadians(180));

    private PathChain shootFirst, toFirstLine, pickFirstLine, cheefFirst, shootSecond,
            driveToSecond, intakeSecondChain, twistSecond, shootSecondLine;

    public void buildPaths(){
        shootFirst = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        toFirstLine = follower.pathBuilder().addPath(new BezierLine(shootPose, firstLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLine.getHeading())
                .build();
        pickFirstLine = follower.pathBuilder().addPath(new BezierLine(firstLine, intakeFirst))
                .setLinearHeadingInterpolation(firstLine.getHeading(), intakeFirst.getHeading())
                .build();
        cheefFirst = follower.pathBuilder().addPath(new BezierLine(intakeFirst, cheefFirstWall))
                .setLinearHeadingInterpolation(intakeFirst.getHeading(), cheefFirstWall.getHeading())
                .build();
        shootSecond = follower.pathBuilder().addPath(new BezierLine(intakeFirst, shootPose))
                .setLinearHeadingInterpolation(intakeFirst.getHeading(), shootPose.getHeading())
                .build();
        driveToSecond = follower.pathBuilder().addPath(new BezierLine(shootPose, secondLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondLine.getHeading())
                .build();
        intakeSecondChain = follower.pathBuilder().addPath(new BezierLine(secondLine, intakeSecond))
                .setLinearHeadingInterpolation(secondLine.getHeading(), intakeSecond.getHeading())
                .build();
        twistSecond = follower.pathBuilder().addPath(new BezierLine(intakeSecond, secondTwist))
                .setLinearHeadingInterpolation(intakeSecond.getHeading(), secondTwist.getHeading())
                .build();
        shootSecondLine = follower.pathBuilder().addPath(new BezierCurve(secondTwist, new Pose(47, 52),shootPose))
                .setLinearHeadingInterpolation(secondTwist.getHeading(), shootPose.getHeading())
                .build();
    }

    private RunCommand updatePoses = new RunCommand(() -> updatedLocalizer.updatePoses());
    private RunCommand updateDistance = new RunCommand(()-> distanceToGoal = updatedLocalizer.getDistance());
    private RunCommand trackTurret = new RunCommand(() -> turret.turretTrack(updatedLocalizer.getFTCPose()));
    private RunCommand runHood = new RunCommand(()-> hood.runHoodRegression(distanceToGoal));
    //private InstantCommand runIntake = new InstantCommand(()-> intake.runIntake(1));
    //private InstantCommand stopIntake = new InstantCommand(()-> intake.runIntake(0));
    //private InstantCommand toggleShooter = new InstantCommand(()-> shooterOn = !shooterOn);

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
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, shootFirst, true),
                        new WaitCommand(750), new InstantCommand(()-> intake.runIntake(1)), new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, toFirstLine),
                        new InstantCommand(()-> intake.runIntake(1)),
                        new FollowPathCommand(follower, pickFirstLine),
                        new FollowPathCommand(follower, cheefFirst),
                        new WaitCommand(500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new FollowPathCommand(follower, shootSecond, true),
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(1)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, driveToSecond),
                        new InstantCommand(()-> intake.runIntake(1)),
                        new FollowPathCommand(follower, intakeSecondChain),
                        new FollowPathCommand(follower, twistSecond),
                        new WaitCommand(500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new FollowPathCommand(follower, shootSecondLine, true),
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new WaitCommand(750),
                        new InstantCommand(()-> intake.runIntake(1)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn)
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
