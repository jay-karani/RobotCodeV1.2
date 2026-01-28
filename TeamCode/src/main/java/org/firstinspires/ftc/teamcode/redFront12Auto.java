package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class redFront12Auto extends CommandOpMode {
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

    public final Pose startPose = RobotConstants.redFrontStart;
    public final Pose shootPose = new Pose(101, 104, Math.toRadians(0));
    public final Pose firstLine = new Pose(96, 84, Math.toRadians(20));
    public final Pose intakeFirst = new Pose(125, 84, Math.toRadians(20));
    public final Pose cheefFirstWall = new Pose(126,88, Math.toRadians(0));
    public final Pose emptyGate = new Pose(127.5, 77, Math.toRadians(90));
    public final Pose secondLine = new Pose(96, 65, Math.toRadians(340));
    public final Pose intakeSecond = new Pose(132.2, 65, Math.toRadians(340));
    public final Pose secondTwist = new Pose (133.2, 59, Math.toRadians(0));
    public final Pose thirdLine = new Pose(96, 42.5, Math.toRadians(340));
    public final Pose intakeThird = new Pose(131.7, 42.5, Math.toRadians(340));
    public final Pose thirdTwist = new Pose(132.7, 34, Math.toRadians(0));
    public final Pose parkPose = new Pose(104, 76, Math.toRadians(90));

    private PathChain shootPreload, toFirstLine, pickFirstLine, cheefFirst, goEmptyGate, shootFirstLine,
            driveToSecond, intakeSecondChain, twistSecond, shootSecondLine,
    driveToThird, intakeThirdChain, twistThird, shootThirdLine, park;

    public void buildPaths(){
        shootPreload = follower.pathBuilder().addPath(new BezierLine(startPose, shootPose))
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
        goEmptyGate = follower.pathBuilder().addPath(new BezierCurve(cheefFirstWall,
                        new Pose(110, 83),emptyGate))
                .setLinearHeadingInterpolation(cheefFirstWall.getHeading(), emptyGate.getHeading())
                .build();
        shootFirstLine = follower.pathBuilder().addPath(new BezierLine(emptyGate, shootPose))
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
        shootSecondLine = follower.pathBuilder().addPath(new BezierCurve(secondTwist, new Pose(97, 52),shootPose))
                .setLinearHeadingInterpolation(secondTwist.getHeading(), shootPose.getHeading())
                .build();
        driveToThird = follower.pathBuilder().addPath(new BezierLine(shootPose, thirdLine))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdLine.getHeading())
                .build();
        intakeThirdChain = follower.pathBuilder().addPath(new BezierLine(thirdLine, intakeThird))
                .setLinearHeadingInterpolation(thirdLine.getHeading(), intakeThird.getHeading())
                .build();
        twistThird = follower.pathBuilder().addPath(new BezierLine(intakeThird, thirdTwist))
                .setLinearHeadingInterpolation(intakeThird.getHeading(), thirdTwist.getHeading())
                .build();
        shootThirdLine = follower.pathBuilder().addPath(new BezierCurve(thirdTwist, new Pose(100, 44),shootPose))
                .setLinearHeadingInterpolation(thirdTwist.getHeading(), shootPose.getHeading())
                .build();
        park = follower.pathBuilder().addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    private RunCommand updatePoses = new RunCommand(() -> updatedLocalizer.updatePoses());
    private RunCommand updateDistance = new RunCommand(()-> distanceToGoal = updatedLocalizer.getDistance());
    private RunCommand trackTurret = new RunCommand(() -> turret.turretTrack(updatedLocalizer.getFTCPose()));
    private RunCommand runHood = new RunCommand(()-> hood.runHoodRegression(distanceToGoal));
    private RunCommand updateRobotPose = new RunCommand(()-> RobotConstants.cachedRobotPose = follower.getPose());

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
        RobotConstants.cachedRobotPose = new Pose(0, 0, 0);
        RobotConstants.robotTeam = RobotConstants.Team.RED;
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
        turret.resetTurretEncoder();

        buildPaths();

        /* shooting order
            new InstantCommand(()-> shooterOn = !shooterOn),
                        new WaitCommand(650),
                        new InstantCommand(()-> intake.runIntake(0.8)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn)
         */

        schedule(
                new RunCommand(()-> follower.update()), updatePoses, updateDistance,
                trackTurret, runHood, runShooter, updateRobotPose,
                new SequentialCommandGroup(
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, shootPreload, true),
                        new WaitCommand(500), new InstantCommand(()-> intake.runIntake(0.8)), new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, toFirstLine), new InstantCommand(()-> intake.runIntake(1)),
                        new FollowPathCommand(follower, pickFirstLine), new FollowPathCommand(follower, cheefFirst),
                        new WaitCommand(10), new InstantCommand(()-> intake.runIntake(0)),
                        new FollowPathCommand(follower, goEmptyGate, true),
                        new WaitCommand(500),
                        new ParallelCommandGroup(new FollowPathCommand(follower, shootFirstLine), new SequentialCommandGroup(
                                new WaitCommand(100), new InstantCommand(()-> shooterOn = !shooterOn)
                        )),
                        new WaitCommand(650),
                        new InstantCommand(()-> intake.runIntake(0.8)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, driveToSecond), new InstantCommand(()-> intake.runIntake(1)),
                        new FollowPathCommand(follower, intakeSecondChain), new FollowPathCommand(follower, twistSecond),
                        new WaitCommand(10), new InstantCommand(()-> intake.runIntake(0)),
                        new ParallelCommandGroup(new FollowPathCommand(follower, shootSecondLine), new SequentialCommandGroup(
                                new WaitCommand(300), new InstantCommand(()-> shooterOn = !shooterOn)
                        )),
                        new WaitCommand(650),
                        new InstantCommand(()-> intake.runIntake(0.8)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),

                        new FollowPathCommand(follower, driveToThird), new InstantCommand(()-> intake.runIntake(1)),
                        new FollowPathCommand(follower, intakeThirdChain), new FollowPathCommand(follower, twistThird),
                        new WaitCommand(10), new InstantCommand(()-> intake.runIntake(0)),
                        new ParallelCommandGroup(new FollowPathCommand(follower, shootThirdLine), new SequentialCommandGroup(
                                new WaitCommand(400), new InstantCommand(()-> shooterOn = !shooterOn)
                        )),
                        new WaitCommand(650),
                        new InstantCommand(()-> intake.runIntake(0.8)),
                        new WaitCommand(1500),
                        new InstantCommand(()-> intake.runIntake(0)),
                        new InstantCommand(()-> shooterOn = !shooterOn),
                        new FollowPathCommand(follower, park, true)
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
