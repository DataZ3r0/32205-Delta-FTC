package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    AprilVision s_aprilVision;
    //Intake s_intake;
    //Shooter s_shooter;

    AutoAlignCommand autoAlign;

    @Override
    public void runOpMode() {

        gamepad = new GamepadEx(gamepad1);

        s_drivetrain = new Drivetrain(hardwareMap);
        s_aprilVision = new AprilVision(hardwareMap);
        //s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        CommandScheduler.getInstance().run();

        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.drive(
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    gamepad.getRightX()
            );

            s_aprilVision.getAprilTagData(telemetry);

            aPressed = gamepad.isDown(GamepadKeys.Button.A);
            xPressed = gamepad.isDown(GamepadKeys.Button.X);

            gamepad.readButtons();

            if (xPressed) {
                s_drivetrain.resetYaw();
            }

            if(aPressed) {
                autoAlign = new AutoAlignCommand(s_drivetrain, s_aprilVision);
            }


            s_drivetrain.periodic(telemetry);
            telemetry.addData("A button: ", aPressed);
            telemetry.update();
        }
    }
}
