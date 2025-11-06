package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Turret;


public class GlobalPoseEstimation extends SubsystemBase {
    private OTOS s_otos;
    private AprilVision s_vision;
    private Turret s_turret;

    private SparkFunOTOS.Pose2D pose;
    private double x;
    private double y;
    private double r;
    private double deltaX;
    private double deltaY;

    public GlobalPoseEstimation(OTOS s_otos, AprilVision s_vision, Turret s_turret) {
        this.s_otos = s_otos;
        this.s_vision = s_vision;
        this.s_turret = s_turret;
    }

    public void estimatePose() {
        if (s_vision.foundTarget()) {
            deltaX = s_vision.getTargetRange() * Math.cos(Math.toRadians(s_turret.getTurretAngle() - s_otos.getH()));
            deltaY = s_vision.getTargetRange() * Math.sin(Math.toRadians(s_turret.getTurretAngle() - s_otos.getH()));
            x = Constants.toggles.blueTeam ? Constants.FieldConstants.blueGoal.x - deltaX : Constants.FieldConstants.redGoal.x - deltaX;
            y = Constants.toggles.blueTeam ? Constants.FieldConstants.blueGoal.y - deltaY : Constants.FieldConstants.redGoal.y - deltaY;
            r = s_otos.getH() + s_turret.getTurretAngle() - s_vision.getTargetYaw();
        } else {
            x = s_otos.getX();
            y = s_otos.getY();
            r = s_otos.getH();
        }
        pose = new SparkFunOTOS.Pose2D(x,y,r);
    }

    public SparkFunOTOS.Pose2D getPose() {
        return pose;
    }

    @Override
    public void periodic() {
        estimatePose();
    }
}
