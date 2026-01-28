package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsys;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsys;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsys;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsys;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsys;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;
import org.firstinspires.ftc.teamcode.subsystems.UpdatedLocalizerSubsys;

@TeleOp
public class redTeleOp extends OpMode {
    private Follower follower;
    private TelemetryData telemetryData = new TelemetryData(telemetry);
    private UpdatedLocalizerSubsys updatedLocalizer;
    double turretOffset = 0;
    double flywheelOffset = 0;
    double driveOffset = 0;


    private FlywheelSubsys flywheel;
    private HoodSubsys hood;
    private StopperSubsys stopper;
    private TurretSubsys turret;
    private IntakeSubsys intake;
    private DriveSubsys drivetrain;
    public double distanceToGoal = 0;
    public boolean shooterOn = false;

    @Override
    public void init() {
        RobotConstants.robotTeam = RobotConstants.Team.RED;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(RobotConstants.cachedRobotPose);
        follower.setPose(RobotConstants.cachedRobotPose);
        follower.update();
        updatedLocalizer = new UpdatedLocalizerSubsys(hardwareMap, follower);
        updatedLocalizer.updatePoses();

        flywheel = new FlywheelSubsys(hardwareMap);
        hood = new HoodSubsys(hardwareMap);
        stopper = new StopperSubsys(hardwareMap);
        turret = new TurretSubsys(hardwareMap);
        intake = new IntakeSubsys(hardwareMap);
        drivetrain = new DriveSubsys(hardwareMap);
        stopper.stopperClose();
    }

    @Override
    public void loop(){
        follower.update();
        updatedLocalizer.updatePoses();
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rot = gamepad1.right_stick_x;
        double heading = updatedLocalizer.getFTCHeading();
        distanceToGoal = updatedLocalizer.getDistance() + flywheelOffset;

        drivetrain.runDrive(y, x, rot, heading + driveOffset);

        turret.turretTrack(updatedLocalizer.getFTCPose());
        hood.runHoodRegression(distanceToGoal);

        if (gamepad1.rightBumperWasPressed()){
            shooterOn = !shooterOn;
        }

        if (shooterOn){
            if (stopper.getPos() != RobotConstants.stopperOpen){
                stopper.stopperOpen();
            }
            flywheel.runFlywheelRegression(distanceToGoal);
        } else {
            flywheel.idleFlywheel();
            if (stopper.getPos() != RobotConstants.stopperClose) {
                stopper.stopperClose();
            }
        }

        if (gamepad1.right_trigger >= 0.1){
            intake.runIntake(1);
        } else if (gamepad1.left_trigger >= 0.2){
            intake.runIntake(-gamepad1.left_trigger);
        } else {
            intake .runIntake(0);
        }

        if (gamepad1.dpadLeftWasPressed()){
            turret.changeOffset(-5);
        } else if (gamepad1.dpadRightWasPressed()){
            turret.changeOffset(5);
        }

        if (gamepad1.dpadUpWasPressed()){
            flywheelOffset += 3;
        } else if (gamepad1.dpadDownWasPressed()){
            flywheelOffset -= 3;
        }

        if (gamepad1.squareWasPressed()){
            driveOffset -= 5;
        } else if (gamepad1.circleWasPressed()){
            driveOffset += 5;
        }
    }
}
