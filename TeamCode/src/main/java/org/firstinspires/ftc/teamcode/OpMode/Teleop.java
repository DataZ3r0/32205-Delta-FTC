package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx gamepad;

    boolean aPressed;
    boolean xPressed;

    Drivetrain s_drivetrain;
    //Intake s_intake;
    //Shooter s_shooter;

    @Override
    public void runOpMode() {

        gamepad = new GamepadEx(gamepad1);

        s_drivetrain = new Drivetrain(hardwareMap);
        //s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.drive(
                    gamepad.getLeftY(),
                    gamepad.getLeftX(),
                    gamepad.getRightX()
            );

            aPressed = gamepad.isDown(GamepadKeys.Button.A);
            xPressed = gamepad.isDown(GamepadKeys.Button.X);

            gamepad.readButtons();

            s_drivetrain.periodic(telemetry);
            telemetry.addData("A button: ", aPressed);
            telemetry.update();
        }
    }
}
