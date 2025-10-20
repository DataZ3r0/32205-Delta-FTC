package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Odometry.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrains.TeleopMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx driverGamepad;
    GamepadEx opGamepad;
    MultipleTelemetry m_telemetry;

    TeleopMecanum s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    //Shooter s_shooter;
    OTOSLocalizer s_otos;

    boolean intakeReversed;

    @Override
    public void runOpMode() {

        driverGamepad = new GamepadEx(gamepad1);
        opGamepad = new GamepadEx(gamepad2);

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s_drivetrain = new TeleopMecanum(hardwareMap);
        s_aprilVision = new AprilVision(hardwareMap);
        s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        s_otos = new OTOSLocalizer(hardwareMap, new Pose2d(0, 0, 0));

        intakeReversed = false;

        CommandScheduler.getInstance().run();

        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.drive(
                    driverGamepad.getLeftY(),
                    -driverGamepad.getLeftX(),
                    -driverGamepad.getRightX()
            );

            s_aprilVision.getAprilTagData(m_telemetry);

            driverGamepad.readButtons();
            opGamepad.readButtons();

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)){
                s_drivetrain.resetYaw();
            }

            if(opGamepad.isDown(GamepadKeys.Button.A)) {
                s_intake.outtake();
            } else {
                s_intake.intake();
            }
            if(opGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if(s_intake.getPower() > 0.001) {
                    s_intake.stop();
                } else {
                    s_intake.run();
                }
            }

            s_drivetrain.periodic(m_telemetry);
            s_otos.update();
            s_aprilVision.getAprilTagData(m_telemetry);
            m_telemetry.update();
        }
    }
}
