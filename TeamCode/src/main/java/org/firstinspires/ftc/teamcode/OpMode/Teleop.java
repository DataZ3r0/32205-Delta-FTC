package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Subsystems.CameraStream;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Delta", group="Teleop")
public class Teleop extends LinearOpMode {

    GamepadEx gamepad;

    boolean aPressed;
    boolean xPressed;

    Drivetrain s_drivetrain;
    AprilTagDetection s_aprilTagVision;
    CameraStream s_CameraStream;

    //Intake s_intake;
    //Shooter s_shooter;

    @Override
    public void runOpMode() {

        gamepad = new GamepadEx(gamepad1);

        s_drivetrain = new Drivetrain(hardwareMap);
        s_aprilTagVision = new AprilTagDetection(hardwareMap);
        //s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

//        s_CameraStream = new CameraStream();
//
//        new VisionPortal.Builder()
//                .addProcessor(s_CameraStream)
//                .setCamera(BuiltinCameraDirection.BACK)
//                .build();
//
//        FtcDashboard.getInstance().startCameraStream(s_CameraStream, 60);

        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.drive(
                    gamepad.getLeftY(),
                    -gamepad.getLeftX(),
                    gamepad.getRightX()
            );

            s_aprilTagVision.getAprilTagData(telemetry);

            aPressed = gamepad.isDown(GamepadKeys.Button.A);
            xPressed = gamepad.isDown(GamepadKeys.Button.X);

            gamepad.readButtons();

            if (xPressed) {
                s_drivetrain.resetYaw();
            }


            s_drivetrain.periodic(telemetry);
            telemetry.addData("A button: ", aPressed);
            telemetry.update();
        }
    }
}
