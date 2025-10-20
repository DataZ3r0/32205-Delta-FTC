package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.DriveToPoint;
import org.firstinspires.ftc.teamcode.Odometry.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrains.TeleopMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Autonomous(name="Delta", group="Auto")
public class Auto extends LinearOpMode {

    MultipleTelemetry m_telemetry;

    TeleopMecanum a_drivetrain;
    AprilVision a_aprilVision;
    Intake a_intake;
//    Shooter a_shooter;
    OTOSLocalizer a_otos;

    DriveToPoint c_driveToPoint;

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

        a_drivetrain = new TeleopMecanum(hardwareMap);
        a_aprilVision = new AprilVision(hardwareMap);
//        a_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        a_otos = new OTOSLocalizer(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        c_driveToPoint = new DriveToPoint(a_drivetrain, a_otos, new SparkFunOTOS.Pose2D(30, 30, 90));
        c_driveToPoint.execute();

//        while(opModeIsActive()) {
//            a_aprilVision.getAprilTagData(m_telemetry);
//        }
    }
}