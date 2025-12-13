package org.firstinspires.ftc.teamcode.Commands;

import static java.lang.Runtime.getRuntime;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    double timestamp;

    public AutoDrive(Drivetrain s_drivetrain, OTOS s_otos){
        this.s_drivetrain = s_drivetrain;
        this.s_otos = s_otos;

        xController = new PIDController(Constants.DrivetrainConstants.drivePID.drivekP,
                Constants.DrivetrainConstants.drivePID.drivekI,
                Constants.DrivetrainConstants.drivePID.drivekD);
        yController = new PIDController(Constants.DrivetrainConstants.drivePID.drivekP,
                Constants.DrivetrainConstants.drivePID.drivekI,
                Constants.DrivetrainConstants.drivePID.drivekD);
        rotationController = new PIDController(Constants.DrivetrainConstants.drivePID.turnkP,
                Constants.DrivetrainConstants.drivePID.turnkI,
                Constants.DrivetrainConstants.drivePID.turnkD);
    }

    public void init() {
        isFinished = false;
        s_otos.setPose(new SparkFunOTOS.Pose2D(0,0, s_drivetrain.getHeading()));
        timestamp = System.nanoTime();
    }
    public void run(SparkFunOTOS.Pose2D targetPose, double maxLinearSpeed, double maxRotSpeed, MultipleTelemetry m_telemetry) {

        isFinished = false;

        xError = targetPose.x - s_otos.getX();
        yError = targetPose.y - s_otos.getY();
        rError = targetPose.h - s_otos.getH();

        xOutput = MathUtil.clamp(xController.calculate(s_otos.getX(), targetPose.x), -maxLinearSpeed, maxLinearSpeed);
        yOutput = MathUtil.clamp(yController.calculate(s_otos.getY(), targetPose.y), -maxLinearSpeed, maxLinearSpeed);
        rOutput = MathUtil.clamp(rotationController.calculate(s_otos.getH(), targetPose.h), -maxRotSpeed, maxRotSpeed);


        s_drivetrain.drive(yOutput, xOutput , rOutput);

        if (Math.abs(xError) < Constants.DrivetrainConstants.driveTolerance &&
                Math.abs(yError) < Constants.DrivetrainConstants.driveTolerance &&
                Math.abs(rError) < Constants.DrivetrainConstants.rotationTolerance) {
            s_drivetrain.stop();
            isFinished = true;
        } else if (System.nanoTime() > timestamp + (5 * Math.pow(10, 9))) {
            isFinished = true;
            s_drivetrain.stop();
        }

        m_telemetry.addData("xOutput", xOutput);
        m_telemetry.addData("yOutput", yOutput);
        m_telemetry.addData("rOutput", rOutput);
    }

    public boolean isFinished() {
        return isFinished;
    }

}
