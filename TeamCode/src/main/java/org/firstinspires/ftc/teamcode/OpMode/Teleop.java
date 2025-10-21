package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx driverGamepad;
    GamepadEx opGamepad;
    MultipleTelemetry m_telemetry;

    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    //Shooter s_shooter;
    OTOS s_otos;

    GamepadKeys.Trigger intakeTrigger;
    GamepadKeys.Trigger outtakeTrigger;

    boolean intakeReversed;

    @Override
    public void runOpMode() {

        driverGamepad = new GamepadEx(gamepad1);
        opGamepad = new GamepadEx(gamepad2);

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s_drivetrain = new Drivetrain(hardwareMap);
        s_aprilVision = new AprilVision(hardwareMap);
        s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        s_otos = new OTOS(hardwareMap, m_telemetry);

        intakeTrigger = GamepadKeys.Trigger.RIGHT_TRIGGER;
        outtakeTrigger = GamepadKeys.Trigger.LEFT_TRIGGER;

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


            s_drivetrain.periodic(m_telemetry);
            s_otos.periodic(m_telemetry);
            s_aprilVision.getAprilTagData(m_telemetry);
            m_telemetry.update();
        }
    }
}
