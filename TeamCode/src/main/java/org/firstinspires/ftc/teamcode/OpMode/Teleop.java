package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Delta", group="Teleop")

public class Teleop extends LinearOpMode {

    GamepadEx gamepad;

    boolean aPressed;
    boolean xPressed;

    Drivetrain s_drivetrain;
    AprilVision s_aprilTagVision;
    //Intake s_intake;
    //Shooter s_shooter;


    @Override
    public void runOpMode() {

        gamepad = new GamepadEx(gamepad1);

        s_drivetrain = new Drivetrain(hardwareMap);
        s_aprilTagVision = new AprilVision(hardwareMap);
        //s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        AutoAlignCommand autoAlign = new AutoAlignCommand(s_drivetrain, s_aprilTagVision);
        CommandScheduler.getInstance().registerSubsystem(s_drivetrain, s_aprilTagVision);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            s_drivetrain.drive(
//                    0,0, 1
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    gamepad.getRightX()
            );

            s_aprilTagVision.getAprilTagData(telemetry);

            if(s_aprilTagVision.findTarget()) {
                gamepad.getGamepadButton(GamepadKeys.Button.A).
                        whenPressed(autoAlign);
            }

            xPressed = gamepad.isDown(GamepadKeys.Button.X);

            gamepad.readButtons();

            if (xPressed) {
                s_drivetrain.resetYaw();
            }

            while (aPressed && s_aprilTagVision.findTarget()) {
                CommandScheduler.getInstance().schedule(autoAlign);
            }

            s_drivetrain.periodic(telemetry);
            telemetry.update();
        }
    }
}
