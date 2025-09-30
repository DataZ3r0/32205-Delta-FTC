package org.firstinspires.ftc.teamcode.OpMode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous (name = "DeltaAuto", group = "Autonomous")
public class firstAuto extends LinearOpMode {

    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    //Intake s_intake;
    //Shooter s_shooter;



    AlignToTagCommand autoAlign;

    @Override
    public void runOpMode() {
        s_drivetrain = new Drivetrain(hardwareMap, true);
        s_aprilVision = new AprilVision(hardwareMap);
        //s_intake = new Intake(hardwareMap);
        //s_shooter = new Shooter(hardwareMap);

        CommandScheduler.getInstance().run();

        waitForStart();

        s_drivetrain.StraightForward(Constants.DrivetrainConstants.maxDrive, 6, 0);
    }
}
