package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx driverGamepad;
    GamepadEx opGamepad;
    MultipleTelemetry m_telemetry;

    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    Shooter s_shooter;
    OTOS s_otos;

    GamepadKeys.Trigger intakeTrigger;
    GamepadKeys.Trigger outtakeTrigger;
    GamepadKeys.Button shooterButton;

    boolean intakeReversed;

    @Override
    public void runOpMode() {

        driverGamepad = new GamepadEx(gamepad1);
        opGamepad = new GamepadEx(gamepad2);

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry);
        s_intake = new Intake(hardwareMap);
        s_shooter = new Shooter(hardwareMap);

        s_otos = new OTOS(hardwareMap, m_telemetry);

        intakeTrigger = GamepadKeys.Trigger.RIGHT_TRIGGER;
        outtakeTrigger = GamepadKeys.Trigger.LEFT_TRIGGER;

        shooterButton = GamepadKeys.Button.X;

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

            if (opGamepad.getTrigger(intakeTrigger) > 0.001) {
                s_intake.runIntake(opGamepad.getTrigger(intakeTrigger));
            } else if (opGamepad.getTrigger(outtakeTrigger) > 0.001) {
                s_intake.runOuttake(opGamepad.getTrigger(outtakeTrigger));
            } else {
                s_intake.stop();
            }
            if(driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
                s_otos.resetOTOS();
            }
//            if(opGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                if(s_intake.getPower() > 0.001) {
//                    s_intake.stop();
//                } else {
//                    s_intake.run();
//                }
//            }

            if (opGamepad.isDown(shooterButton)) {
                s_shooter.runShooter(1.0);
            } else {
                s_shooter.stop();
            }

            m_telemetry.update();
        }
    }
}
