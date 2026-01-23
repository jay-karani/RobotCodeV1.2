package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsys;

@Configurable
@TeleOp
public class TurretTestOpMode extends OpMode {
    private TurretSubsys turret;
    private static double turretTarget = 0;

    @Override
    public void init() {
        turret = new TurretSubsys(hardwareMap);
    }

    @Override
    public void loop() {
        turret.turretToAngle(turretTarget);
        turret.updateConstants();
        telemetry.addData("current", turret.getTurretMotor().motor.getCurrentPosition());
        telemetry.addData("target", turret.getTurretMotor().motor.getTargetPosition());

    }
}
