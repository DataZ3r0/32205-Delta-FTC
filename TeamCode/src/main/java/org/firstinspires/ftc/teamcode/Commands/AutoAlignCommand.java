package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class AutoAlignCommand extends CommandBase {
    private Drivetrain s_drivetrain;
    private AprilVision s_tagDetection;

    final double DESIRED_DISTANCE = 12.0;

    final double SPEED_GAIN = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.01;
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;
    private AprilVision desiredTag = null;
    private boolean tagFound;
    double driveX;
    double driveY;
    double rotation;
    double rangeError;
    double headingError;
    double yawError;


    public AutoAlignCommand(Drivetrain drivetrain, AprilVision s_tagDetection) {
        this.s_drivetrain = drivetrain;
        this.s_tagDetection = s_tagDetection;

        addRequirements(s_drivetrain, s_tagDetection);
    }

    @Override
    public void initialize() {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double driveX = 0;
        double driveY = 0;
        double rotation = 0;
    }

    @Override
    public void execute() {
        //driveY = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        //driveX = Range.clip(yawError * SPEED_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        rotation = Range.clip(headingError * SPEED_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        s_drivetrain.drive(driveY, driveX, rotation);
    }

//    @Override
//    public void end(boolean interrupted) {
//        s_drivetrain.stop(); // Stop the motors
//    }

}
