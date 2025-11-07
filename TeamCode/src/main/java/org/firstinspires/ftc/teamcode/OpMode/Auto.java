package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.VisionStates;

@Autonomous(name="Delta", group="Auto")
public class Auto extends LinearOpMode {

    MultipleTelemetry m_telemetry;

    Drivetrain a_drivetrain;
    AprilVision a_aprilVision;
    OTOS a_otos;
    int state;

    VisionStates visionState;

    SparkFunOTOS.Pose2D currentPose;
    private SparkFunOTOS.Pose2D targetPose;
    double xError;
    double yError;
    double hError;
    double xTarget;
    double yTarget;
    double hTarget;

    @Override
    public void runOpMode() {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        a_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        a_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState);
//       a_intake = new Intake(hardwareMap);
//        s_shooter = new Shooter(hardwareMap);
    a_otos = new OTOS(hardwareMap, m_telemetry);

        waitForStart();

        while(opModeIsActive()) {
//            driveToPoint(new SparkFunOTOS.Pose2D(10,10,0));
//            a_aprilVision.getAprilTagData(m_telemetry);
        }
    }


//    public void driveToPoint(SparkFunOTOS.Pose2D targetPose) {
//        while (opModeIsActive()) {
//            currentPose = a_otos.getPose();
//
//            double xError = targetPose.x - currentPose.x;
//            double yError = targetPose.y - currentPose.y;
//            Rotation2d angleError = new Rotation2d(targetPose.h).minus(new Rotation2d(currentPose.h));
//            double hError = angleError.getRadians();
//            double hError = targetPose.h - currentPose.h;
//
//            double xPower = xError * Constants.DrivetrainConstants.drivePID.kPdrive;
//            double yPower = yError * Constants.DrivetrainConstants.drivePID.kPstrafe;
//            double hPower = hError * Constants.DrivetrainConstants.drivePID.kPturn;
//
//            a_drivetrain.drive(yPower, xPower, hPower);
//
//            m_telemetry.addData("Target X", targetPose.x);
//            m_telemetry.addData("Target Y", targetPose.y);
//            m_telemetry.addData("Current X", currentPose.x);
//            m_telemetry.addData("Current Y", currentPose.y);
//            m_telemetry.addData("xError", xError);
//            m_telemetry.addData("yError", yError);
//            m_telemetry.addData("hError", hError);
//            m_telemetry.update();
//
//            if (Math.hypot(xError, yError) < 0.3 && Math.abs(hError) < Math.toRadians(3)) {
//                a_drivetrain.stop();
//                break;
//            }
//        }
//    }
}