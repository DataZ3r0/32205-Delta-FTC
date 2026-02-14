package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.NewTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;

public class TurretCalculator {
    private OTOS s_OTOS;
    private AprilVision s_AprilVision;
    private NewTurret s_turret;

    public TurretCalculator(OTOS s_OTOS, AprilVision s_AprilVision, NewTurret s_turret) {
        this.s_OTOS = s_OTOS;
        this.s_AprilVision = s_AprilVision;
        this.s_turret = s_turret;
    }

    public void correctOTOS() {
        Pose3D robotPose = s_AprilVision.getRobotPose();
        Position robotPoseInches = robotPose.getPosition().toUnit(DistanceUnit.INCH);
        s_OTOS.setPose(new SparkFunOTOS.Pose2D(robotPoseInches.x,
                robotPoseInches.y,
                robotPose.getOrientation().getYaw(AngleUnit.DEGREES) + s_turret.getRobotTurretAngle()));
    }

    public SparkFunOTOS.Pose2D getOTOSPoseBlue() {
        return new SparkFunOTOS.Pose2D(
                s_OTOS.getY(), -s_OTOS.getX(), s_OTOS.getH()
        );
    }

    public SparkFunOTOS.Pose2D getOTOSPoseRed() {
        return new SparkFunOTOS.Pose2D(
                -s_OTOS.getY(), s_OTOS.getX(), s_OTOS.getH()
        );
    }


    //returns field-relative setpoint to aim turret
    public double getTurretSetpointBlue(SparkFunOTOS.Pose2D input) {
        SparkFunOTOS.Pose2D robotPose = getOTOSPoseBlue();
        return wrapAngle(Math.toDegrees(Math.atan2(input.y - robotPose.y, input.x - robotPose.x)) - 90);
    }

    public double getTurretSetpointRed(SparkFunOTOS.Pose2D input) {
        SparkFunOTOS.Pose2D robotPose = getOTOSPoseRed();
        return wrapAngle(Math.toDegrees(Math.atan2(input.y - robotPose.y, input.x - robotPose.x)) + 90);
    }

    public double getDistanceFromTarget(SparkFunOTOS.Pose2D robotPose, SparkFunOTOS.Pose2D targetPose) {
        return Math.hypot(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    }

    public double wrapAngle(double angleDeg) {
        angleDeg = angleDeg % 360;     // keep within 0–360 or –360–0

        if (angleDeg > 180)
            angleDeg -= 360;

        if (angleDeg < -180)
            angleDeg += 360;

        return angleDeg;
    }

//    public SparkFunOTOS.Pose2D rotateAroundRobotPose(SparkFunOTOS.Pose2D input) {
//
//    }
}
