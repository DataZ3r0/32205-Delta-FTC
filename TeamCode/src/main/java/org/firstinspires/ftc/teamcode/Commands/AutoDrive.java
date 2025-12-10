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
    public void run(SparkFunOTOS.Pose2D targetPose, double maxLinearSpeed, double maxRotSpeed) {

        isFinished = false;

        xError = targetPose.x - s_otos.getX();
        yError = targetPose.y - s_otos.getY();
        rError = targetPose.h - s_otos.getH();

        if (!(Math.abs(xError) < Constants.DrivetrainConstants.driveTolerance)) {
            xOutput = MathUtil.clamp(xController.calculate(s_otos.getX(), targetPose.x), -maxLinearSpeed, maxLinearSpeed);
        } else {
            xOutput = 0;
        }

        if (!(Math.abs(yError) < Constants.DrivetrainConstants.driveTolerance)) {
            yOutput = MathUtil.clamp(yController.calculate(s_otos.getY(), targetPose.y), -maxLinearSpeed, maxLinearSpeed);
        } else {
            yOutput = 0;
        }

        if (!(Math.abs(rError) < Constants.DrivetrainConstants.rotationTolerance)) {
            rOutput = MathUtil.clamp(rotationController.calculate(s_otos.getH(), targetPose.h), -maxRotSpeed, maxRotSpeed);
        } else {
            rOutput = 0;
        }

        s_drivetrain.drive(yOutput, xOutput , rOutput);

        if (xOutput == 0 && yOutput == 0 && rOutput == 0) {
            s_drivetrain.stop();
            isFinished = true;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }

}
