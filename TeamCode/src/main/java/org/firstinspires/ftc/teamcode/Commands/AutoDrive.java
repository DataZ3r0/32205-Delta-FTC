package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Utilities.MathUtil;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class AutoDrive {
    Drivetrain s_drivetrain;
    OTOS s_otos;
    PIDController xController;
    PIDController yController;
    PIDController rotationController;
    double xError;
    double yError;
    double rError;
    double xOutput;
    double yOutput;
    double rOutput;
    boolean isFinished;

    public AutoDrive(Drivetrain s_drivetrain, OTOS s_otos){
        this.s_drivetrain = s_drivetrain;
        this.s_otos = s_otos;

        xController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.001, 0.0);
        yController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.001, 0.0);
        rotationController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.001, 0.0);
    }

    public void init() {
        isFinished = false;
        s_otos.setPose(new SparkFunOTOS.Pose2D(0,0, s_drivetrain.getHeading()));
    }
    public void run(SparkFunOTOS.Pose2D targetPose) {
        xError = targetPose.x - s_otos.getX();
        yError = targetPose.y - s_otos.getY();
        rError = targetPose.h - s_otos.getH();

        xOutput = MathUtil.clamp(xController.calculate(xError), -0.2, 0.2);
        yOutput = MathUtil.clamp(yController.calculate(xError), -0.2, 0.2);
        rOutput = MathUtil.clamp(rotationController.calculate(rError), -0.2, 0.2);

        s_drivetrain.drive(yOutput, xOutput , rOutput);
        if (xError < Constants.DrivetrainConstants.driveTolerance
                && yError < Constants.DrivetrainConstants.driveTolerance
                && rError < Constants.DrivetrainConstants.rotationTolerance) {
            s_drivetrain.stop();
            isFinished = true;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }

}
