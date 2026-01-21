package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class UpdatedLocalizerSubsys extends SubsystemBase {
    private Limelight3A limelight;
    private Pose pedroPose;
    private Pose2D ftcPose;
    private Follower follower;
    private LLResult latestResult;

    private double goalX = RobotConstants.goalX, goalY;

    public UpdatedLocalizerSubsys(final HardwareMap hwMap, Follower follower){
        this.follower = follower;

        limelight = hwMap.get(Limelight3A.class, RobotConstants.limelightName);
        limelight.start();
        if (RobotConstants.robotTeam == RobotConstants.Team.RED) {
            limelight.pipelineSwitch(RobotConstants.redPipeline);
            goalY = RobotConstants.redGoalY;
        } else {
            limelight.pipelineSwitch(RobotConstants.bluePipeline);
            goalY = RobotConstants.blueGoalY;
        }

    }

    public void updatePoses(){
        pedroPose = follower.getPose();
        ftcPose = RobotConstants.pedroToFTC(pedroPose);
    }

    public double getDistance() {
        double referenceX = ftcPose.getX(DistanceUnit.INCH);
        double referenceY = ftcPose.getY(DistanceUnit.INCH);
        double llX = 0, llY = 0;
        latestResult = limelight.getLatestResult();
        if (latestResult != null && latestResult.isValid()){
            Pose3D botpose = latestResult.getBotpose();
            if (botpose != null){
                llX = botpose.getPosition().x * 100 / 2.54;
                llY = botpose.getPosition().y * 100 / 2.54;
            }
        }
        if (Math.abs(referenceX) > 10 && Math.abs(referenceY) > 10){
            if (Math.signum(referenceX) == -Math.signum(llX)) {llX *= -1;}
            if (Math.signum(referenceY) == -Math.signum(llY)) {llY *= -1;}
        }
        double distX = llX - goalX, distY = llY - goalY;
        return Math.hypot(distX, distY);
    }

    public void forceUpdateFollower(){
        follower.update();
    }

    public double getFTCHeading(){
        return ftcPose.getHeading(AngleUnit.DEGREES);
    }

    public Pose2D getFTCPose(){
        return ftcPose;
    }

    public Pose getPedroPose(){
        return pedroPose;
    }

    public void setFollowerPose(Pose setPose){
        follower.setPose(setPose);
    }

}