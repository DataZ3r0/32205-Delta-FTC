package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.maxDrive;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.maxStrafe;
import static org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants.maxTurn;

import com.arcrobotics.ftclib.command.CommandBase;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class AlignToTagCommand extends CommandBase {
    private final Drivetrain s_drivetrain;
    private final AprilVision s_tagDetection;
    private PIDController drivePID, strafePID, turnPID;

    double driveX;
    double driveY;
    double rotation;
    private final double desiredRange = 30;
    double rangeError;
    double headingError;
    double yawError;


    public AlignToTagCommand(Drivetrain drivetrain, AprilVision s_tagDetection) {
        this.s_drivetrain = drivetrain;
        this.s_tagDetection = s_tagDetection;

        addRequirements(s_drivetrain, s_tagDetection);
    }

    @Override
    public void initialize() {
//        rangeError = AprilVision.getTargetRange() - desiredRange;
//        bearingError = AprilVision.getTargetBearing();
//        yawError = -AprilVision.getTargetYaw();
    }

    @Override
    public void execute() {

        rangeError = AprilVision.getTargetRange() - desiredRange;
        yawError = AprilVision.getTargetYaw();
        headingError = AprilVision.getTargetBearing();

        driveY = Range.clip(rangeError * drivePID.getP(), -maxDrive, maxDrive);
        driveX = Range.clip(yawError * strafePID.getP(), -maxStrafe, maxStrafe);
        rotation = Range.clip(headingError * turnPID.getP(), -maxTurn, maxTurn);

//        s_drivetrain.drive(2, 2, 2);
        s_drivetrain.drive(0, 0, -rotation);
    }


    @Override
    public boolean isFinished() {
        if(s_tagDetection.findTarget()) {
            return Math.abs(headingError) < 0.2;
        } else {
            return true;
        }
//        return false;
    }
    @Override
    public void end(boolean interupted) {

    }



}
