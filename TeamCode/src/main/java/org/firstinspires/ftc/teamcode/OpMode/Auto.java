package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

@Autonomous(name="Delta", group="Auto")
public class Auto extends LinearOpMode {

    MultipleTelemetry m_telemetry;

    Drivetrain a_drivetrain;
    AprilVision a_aprilVision;
    Intake a_intake;
//    Shooter a_shooter;
    OTOS a_otos;

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

        a_drivetrain = new Drivetrain(hardwareMap);
        a_aprilVision = new AprilVision(hardwareMap);
//        a_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        a_otos = new OTOS(hardwareMap, m_telemetry);

        waitForStart();

        while(opModeIsActive()) {
            driveToPoint(new SparkFunOTOS.Pose2D(10,10,0));
            a_aprilVision.getAprilTagData(m_telemetry);
        }
    }

    public void driveToPoint(SparkFunOTOS.Pose2D targetPose) {
        currentPose = a_otos.getPose();

        xError = targetPose.x - currentPose.x;
        yError = targetPose.y - currentPose.y;
        hError = targetPose.h - currentPose.h;

        double xPower = xError;
        double yPower = yError;
        double hPower = hError;

        a_drivetrain.drive(yPower, xPower, hPower);

        if(Math.hypot(xError, yError) < 0.2 && Math.abs(hError) < 0.2) {
            a_drivetrain.stop();
        }
    }
}