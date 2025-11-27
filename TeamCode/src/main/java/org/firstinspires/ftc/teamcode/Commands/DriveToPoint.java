package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.driveTolerance;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.drivekD;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.drivekI;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.drivekP;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.strafeTolerance;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.strafekD;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.strafekI;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.strafekP;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.turnTolerance;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.turnkD;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.turnkI;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.drivePID.turnkP;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Odometry.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrains.TeleopMecanum;

public class DriveToPoint extends CommandBase {
    TeleopMecanum s_drive;
    OTOSLocalizer s_otos;
    PIDController driveController;
    PIDController strafeController;
    PIDController turnController;
    SparkFunOTOS.Pose2D targetPose;
    Pose2d currentPose;
    double xError;
    double yError;
    double hError;
    double xSpeed;
    double ySpeed;
    double hSpeed;
    public DriveToPoint (TeleopMecanum s_drive, OTOSLocalizer s_otos, SparkFunOTOS.Pose2D targetPose) {
        this.s_drive = s_drive;
        this.s_otos = s_otos;
        this.targetPose = targetPose;

        driveController = new PIDController(drivekP, drivekI, drivekD);
        strafeController = new PIDController(strafekP, strafekI, strafekD);
        turnController = new PIDController(turnkP, turnkI, turnkD);

        addRequirements(s_drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentPose = s_otos.getPose();

        xError = targetPose.x - currentPose.position.x;
        yError = targetPose.y - currentPose.position.y;
        hError = targetPose.h - currentPose.heading.toDouble();

        xSpeed = strafeController.calculate(xError);
        ySpeed = driveController.calculate(yError);
        hSpeed = turnController.calculate(hError);

        s_drive.drive(ySpeed, xSpeed, hSpeed);
    }


    @Override
    public boolean isFinished() {
        return xError < strafeTolerance && yError < driveTolerance && hError < turnTolerance;
    }
    @Override
    public void end(boolean interrupted) {
        s_drive.stop();
    }

}
