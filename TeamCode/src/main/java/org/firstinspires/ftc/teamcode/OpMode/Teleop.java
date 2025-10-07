package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx gamepad;

    boolean aPressed;
    boolean xPressed;

    MultipleTelemetry m_telemetry;

    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    //Shooter s_shooter;
    OTOS s_otos;

    boolean intakeReversed;

    AlignToTagCommand autoAlign;

    @Override
    public void runOpMode() {

        gamepad = new GamepadEx(gamepad1);

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s_drivetrain = new Drivetrain(hardwareMap);
        s_aprilVision = new AprilVision(hardwareMap);
        s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        s_otos = new OTOS(hardwareMap, m_telemetry);

        intakeReversed = false;

        CommandScheduler.getInstance().run();

        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.drive(
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    -gamepad.getRightX()
            );

            s_aprilVision.getAprilTagData(telemetry);

            gamepad.wasJustPressed(GamepadKeys.Button.A);
            xPressed = gamepad.isDown(GamepadKeys.Button.X);

            gamepad.readButtons();

            if (xPressed) {
                s_drivetrain.resetYaw();
            }

            if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                s_intake.toggleOuttakeMode(intakeReversed);
                intakeReversed = !intakeReversed;
            }

            if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if(s_intake.getPower() > 0.001) {
                    s_intake.stop();
                } else {
                    s_intake.run();
                }
            }

            s_drivetrain.periodic(m_telemetry);
            s_otos.periodic(m_telemetry);
            s_aprilVision.getAprilTagData(m_telemetry);
            m_telemetry.update();
        }
    }
}
