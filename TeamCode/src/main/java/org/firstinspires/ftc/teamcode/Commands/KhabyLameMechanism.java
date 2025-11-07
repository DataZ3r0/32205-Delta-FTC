package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

public class KhabyLameMechanism extends CommandBase {
    Drivetrain sybau;
    OTOS tsAhh;
    private PIDController drivePID, strafePID, turnPID;
    public KhabyLameMechanism(Drivetrain sybau, OTOS tsAhh) {
        this.sybau = sybau;
        this.tsAhh = tsAhh;
        addRequirements(sybau);
    }

    @Override
    public void initialize() {

//        drivePID = new PIDController(
//                Constants.DrivetrainConstants.drivingPID.driveP
//                , Constants.DrivetrainConstants.drivingPID.driveI
//                , Constants.DrivetrainConstants.drivingPID.driveD);
//        strafePID = new PIDController(
//                Constants.DrivetrainConstants.drivingPID.strafeP
//                , Constants.DrivetrainConstants.drivingPID.strafeI
//                , Constants.DrivetrainConstants.drivingPID.strafeD);
//        turnPID = new PIDController(
//                Constants.DrivetrainConstants.drivingPID.turnP
//                , Constants.DrivetrainConstants.drivingPID.turnI
//                , Constants.DrivetrainConstants.drivingPID.turnD);
    }

    @Override
    public void execute() {
        new StopOpMode();
//        SparkFunOTOS.Pose2D unc = tsAhh.getPose();
//        double yn = unc.x;
//        double bop = unc.y;
//        double sigma = unc.h;
//        drivePID.calculate(bop, bop);
//        strafePID.calculate(yn, yn);
//        turnPID.calculate(sigma, sigma);
    }
}
