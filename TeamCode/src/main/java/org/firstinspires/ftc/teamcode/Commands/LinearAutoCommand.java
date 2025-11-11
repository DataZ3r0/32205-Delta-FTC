package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

import java.util.Timer;

public class LinearAutoCommand extends CommandBase {}
//    private ElapsedTime timer;
//    private double duration;
//    private final Drivetrain a_drivetrain;
//    private final OTOS a_otos;
//    private final MultipleTelemetry m_telemetry;
//    private final SparkFunOTOS.Pose2D targetPose;
//    private double xError, yError, headingError;
//    double drivePow, strafePow, turnPow;
//    private final PIDController drivePID = new PIDController(
//            Constants.DrivetrainConstants.drivingPID.driveP
//            , Constants.DrivetrainConstants.drivingPID.driveI
//            , Constants.DrivetrainConstants.drivingPID.driveD);
//    private final PIDController strafePID = new PIDController(
//            Constants.DrivetrainConstants.drivingPID.strafeP
//            , Constants.DrivetrainConstants.drivingPID.strafeI
//            , Constants.DrivetrainConstants.drivingPID.strafeD);
//    private final PIDController turnPID = new PIDController(
//            Constants.DrivetrainConstants.drivingPID.turnP
//            , Constants.DrivetrainConstants.drivingPID.turnI
//            , Constants.DrivetrainConstants.drivingPID.turnD);
//
//
//    public LinearAutoCommand(Drivetrain a_drivetrain, OTOS a_otos, MultipleTelemetry m_telemetry, SparkFunOTOS.Pose2D targetPose) {
//        this.a_drivetrain = a_drivetrain;
//        this.a_otos = a_otos;
//        this.m_telemetry = m_telemetry;
//        this.targetPose = targetPose;
//
//        addRequirements(a_drivetrain);
//    }
//
//    @Override
//    public void initialize() {
//        drivePID.reset();
//        drivePID.setTolerance(Constants.DrivetrainConstants.drivingPID.tolY);
//        strafePID.reset();
//        strafePID.setTolerance(Constants.DrivetrainConstants.drivingPID.tolX);
//        turnPID.reset();
//        turnPID.setTolerance(Constants.DrivetrainConstants.drivingPID.tolH);
////        a_otos.resetOTOS();
////        timer = new ElapsedTime();
////        timer.reset();
//    }
//    @Override
//    public void execute() {
//        SparkFunOTOS.Pose2D currentPose = a_otos.getPose();
//        yError = targetPose.y - currentPose.y;
//        drivePow = drivePID.calculate(currentPose.y, targetPose.y);
//        xError = targetPose.x - currentPose.x;
//        strafePow = strafePID.calculate(currentPose.x, targetPose.x);
//        headingError = normalizeDegrees(targetPose.h - currentPose.h);
//        turnPow = turnPID.calculate(0, headingError);
////        turnPow = turnPID.calculate(currentPose.h, targetPose.h);
//        a_drivetrain.drive(drivePow, -strafePow, turnPow);
//        periodic(m_telemetry);
//    }
//    @Override
//    public void end(boolean interrupted) {
//        a_drivetrain.stop(a_otos);
//    }
//    @Override
//    public boolean isFinished() {
////        return timer.seconds() >= 5;
////        return (Math.hypot(xError, yError) < 1);
////        return Math.abs(yError) < Constants.DrivetrainConstants.drivingPID.tolY
////                && Math.abs(yError) < Constants.DrivetrainConstants.drivingPID.tolX
////                && Math.abs(yError) < Constants.DrivetrainConstants.drivingPID.tolH;
//        return drivePID.atSetPoint() && strafePID.atSetPoint() && turnPID.atSetPoint();
//    }
//
//    public void periodic(MultipleTelemetry m_telemetry) {
//        m_telemetry.addData("drivePow: ", drivePow);
//        m_telemetry.addData("strafePow: ", strafePow);
//        m_telemetry.addData("turnPow: ", turnPow);
//        m_telemetry.addData("setpoint: ", targetPose);
//        m_telemetry.addData("atYsetpoint: ", drivePID.atSetPoint());
//        m_telemetry.addData("atXsetpoint: ", strafePID.atSetPoint());
//        m_telemetry.addData("atHsetpoint: ", turnPID.atSetPoint());
//    }
//
////      example:
////    p0 = new Vector2(800, 50);
////    p1 = new Vector2(800, 100);
////    p2 = new Vector2(600, 400);
////    p3 = new Vector2(600, 450);
////    curve = new CubicBezierCurve(p0, p1, p2, p3);
//
//
//}
