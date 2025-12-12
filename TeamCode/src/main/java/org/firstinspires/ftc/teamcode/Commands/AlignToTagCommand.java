package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class AlignToTagCommand extends CommandBase {
    private final Drivetrain s_drivetrain;
    private final AprilVision s_tagDetection;

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
//
//        rangeError = AprilVision.getTargetRange() - desiredRange;
//        yawError = s_tagDetection.getTargetYaw();
//        headingError = AprilVision.getTargetBearing();
//
//        driveY = Range.clip(rangeError * drivekP, -maxDrive, maxDrive);
//        driveX = Range.clip(yawError * kPstrafe, -maxStrafe, maxStrafe);
//        rotation = Range.clip(headingError * turnkP, -maxTurn, maxTurn);
//
////        s_drivetrain.drive(2, 2, 2);
//        s_drivetrain.drive(0, 0, -rotation);
    }


    @Override
    public boolean isFinished() {
        if(s_tagDetection.foundTarget()) {
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
