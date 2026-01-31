package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.AutoDrive;
import org.firstinspires.ftc.teamcode.Commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.MiddleStage;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.GlobalPoseEstimation;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.VisionStates;

@Autonomous(name="Delta-FarBLUE", group="Auto")
public class FarAutoBlue extends LinearOpMode {
    MultipleTelemetry m_telemetry;

    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    MiddleStage s_middleStage;
    Shooter s_shooter;
    Turret s_turret;
    OTOS s_otos;
    GlobalPoseEstimation poseEstimation;


    VisionStates visionState;
    distanceSensor s_ds;

    GamepadKeys.Trigger intakeTrigger;
    boolean intakeTriggerPressed;
    GamepadKeys.Trigger outtakeTrigger;
    boolean outtakeTriggerPressed;
    GamepadKeys.Button shooterButton;
    boolean intakeReversed;

    int phase;

    double setpoint;
    double timestamp;
    double output;

    PIDController driveController;
    PIDController rotationController;

    AutoIntakeCommand intakeCommand;

    AutoDrive autoDrive1;
    AutoDrive autoDrive2;
    AutoDrive autoDrive3;
    AutoDrive autoDrive4;
    AutoDrive autoDrive5;
    AutoDrive autoDrive6;
    AutoDrive autoDrive7;
    AutoDrive autoDrive8;
    AutoDrive autoDrive9;
    AutoDrive autoDrive10;
    AutoDrive autoDrive11;
    AutoDrive autoDrive12;

    @Override
    public void runOpMode() {

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionState = new VisionStates();

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState, 20);
        s_intake = new Intake(hardwareMap);
        s_middleStage = new MiddleStage(hardwareMap);
        s_shooter = new Shooter(hardwareMap, m_telemetry, false);
        s_turret = new Turret(hardwareMap, s_drivetrain, m_telemetry);

        s_otos = new OTOS(hardwareMap, m_telemetry);
//        poseEstimation = new GlobalPoseEstimation(s_otos, s_aprilVision, s_turret);
        intakeReversed = false;

        visionState.setState(VisionStates.VisionState.SHOOT);

        intakeCommand = new AutoIntakeCommand(s_intake, s_middleStage, s_shooter);

//        driveController = new PIDController(Constants.DrivetrainConstants.drivePID.drivekP, 0.0, 0.0);
//        rotationController = new PIDController(Constants.DrivetrainConstants.drivePID.drivekP, 0.0, 0.0);

//        CommandScheduler.getInstance().run();
        s_turret.stopTurret();
        s_drivetrain.resetYaw();
        intakeCommand.disable();
//        autoDrive.init();
        s_otos.setPose(new SparkFunOTOS.Pose2D(0,0,0));
//
        timestamp = getRuntime();
        phase = 0;
//        setpoint = 0;


        waitForStart();

        while(opModeIsActive()) {
            s_drivetrain.periodic();
            s_aprilVision.periodic();
            s_shooter.periodic();
            s_turret.periodic();
            s_otos.periodic();

            intakeCommand.periodic();

            if (s_aprilVision.foundTarget()) {
                s_turret.setSetpoint(s_turret.getRobotTurretAngle() + s_aprilVision.getTx());
                s_shooter.setDesiredVelocity(3150);
            } else {
                s_shooter.setSetpoint(1000);
                s_turret.setSetpoint(0);
            }

            s_shooter.stopperPeriodic(null, null, true);

            switch (phase) {
                case 0:
                    if (autoDrive1 == null) {
                        autoDrive1 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive1.init();
                    } else {
                        autoDrive1.run(Constants.AutoConstants.AutoPoints.farautoOne, 0.4, 0.5, m_telemetry);
                        intakeCommand.disable();
                        timestamp = getRuntime();
                        if (autoDrive1.isFinished()) {
                            autoDrive1 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 1:
                    if (s_shooter.atSetpoint()) {
                        intakeCommand.enable();
                    } else {
                        intakeCommand.disable();
                    }
                    if (getRuntime() > timestamp + 7) {
                        phase++;
                        break;
                    }
                    break;
                case 2:
                    if (autoDrive2 == null) {
                        autoDrive2 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive2.init();
                    } else {
                        autoDrive2.run(Constants.AutoConstants.AutoPoints.farautoTwo, 0.4, 0.5, m_telemetry);
                        if (autoDrive2.isFinished()) {
                            autoDrive2 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 3:
                    if (autoDrive3 == null) {
                        autoDrive3 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive3.init();
                    } else {
                        autoDrive3.run(Constants.AutoConstants.AutoPoints.farautoThree, 0.4, 0.5, m_telemetry);
                        intakeCommand.enable();
                        timestamp = getRuntime();
                        if (autoDrive3.isFinished()) {
                            autoDrive3 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 4:
                    if (autoDrive4 == null) {
                        autoDrive4 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive4.init();
                    } else {
                        autoDrive4.run(Constants.AutoConstants.AutoPoints.farautoFour, 0.4, 0.5, m_telemetry);
                        timestamp = getRuntime();
                        if (autoDrive4.isFinished()) {
                            autoDrive4 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 5:
                    if (autoDrive5 == null) {
                        autoDrive5 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive5.init();
                    } else {
                        autoDrive5.run(Constants.AutoConstants.AutoPoints.farautoFive, 0.4, 0.5, m_telemetry);
                        if (s_shooter.atSetpoint()) {
                            intakeCommand.enable();
                        } else {
                            intakeCommand.disable();
                        }
                        if (autoDrive5.isFinished() && getRuntime() > timestamp + 7) {
                            autoDrive5 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 6:
                    if (autoDrive6 == null) {
                        autoDrive6 = new AutoDrive(s_drivetrain, s_otos);
                        autoDrive6.init();
                    } else {
                        autoDrive6.run(Constants.AutoConstants.AutoPoints.farautoSix, 0.4, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive6.isFinished()) {
                            autoDrive6 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 7:
                    s_drivetrain.stop();
                    intakeCommand.disable();
                    break;
            }

            m_telemetry.addData("AUTO PHASE:", phase);
            m_telemetry.update();
        }
    }
}
