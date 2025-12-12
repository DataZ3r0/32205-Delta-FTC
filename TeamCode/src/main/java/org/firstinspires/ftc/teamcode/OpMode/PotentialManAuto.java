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

@Autonomous(name="Delta-12Piece", group="Auto")
public class PotentialManAuto extends LinearOpMode {
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
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState);
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
                s_shooter.setDesiredVelocity(s_aprilVision.getRangeAvg());
            } else {
                s_shooter.setSetpoint(0);
                s_turret.setSetpoint(0);
            }

            s_shooter.stopperPeriodic(null, null);

            switch (phase) {
                case 0:
                    if (autoDrive1 == null) {
                        autoDrive1 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive1.init();
                    } else {
                        autoDrive1.run(Constants.AutoConstants.AutoPoints.autoOne, 0.25, 0.5, m_telemetry);
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
                    intakeCommand.enable();
                    if (getRuntime() > timestamp + 3) {
                        phase++;
                        break;
                    }
                    break;
                case 2:
                    if (autoDrive2 == null) {
                        autoDrive2 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive2.init();
                    } else {
                        autoDrive2.run(Constants.AutoConstants.AutoPoints.autoTwo, 0.25, 0.5, m_telemetry);
                        if (autoDrive2.isFinished()) {
                            autoDrive2 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 3:
                    if (autoDrive3 == null) {
                        autoDrive3 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive3.init();
                    } else {
                        autoDrive3.run(Constants.AutoConstants.AutoPoints.autoThree, 0.25, 0.5, m_telemetry);
                        intakeCommand.disable();
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
                        autoDrive4 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive4.init();
                    } else {
                        autoDrive4.run(Constants.AutoConstants.AutoPoints.autoFour, 0.25, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive4.isFinished() && getRuntime() > timestamp + 3) {
                            autoDrive4 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 5:
                    if (autoDrive5 == null) {
                        autoDrive5 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive5.init();
                    } else {
                        autoDrive5.run(Constants.AutoConstants.AutoPoints.autoFive, 0.25, 0.5, m_telemetry);
                        intakeCommand.disable();
                        timestamp = getRuntime();
                        if (autoDrive5.isFinished()) {
                            autoDrive5 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 6:
                    if (autoDrive6 == null) {
                        autoDrive6 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive6.init();
                    } else {
                        autoDrive6.run(Constants.AutoConstants.AutoPoints.autoSix, 0.25, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive6.isFinished()) {
                            autoDrive6 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 7:
                    if (autoDrive7 == null) {
                        autoDrive7 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive7.init();
                    } else {
                        autoDrive7.run(Constants.AutoConstants.AutoPoints.autoSeven, 0.25, 0.5, m_telemetry);
                        intakeCommand.disable();
                        timestamp = getRuntime();
                        if (autoDrive7.isFinished()) {
                            autoDrive7 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 8:
                    if (autoDrive8 == null) {
                        autoDrive8 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive8.init();
                    } else {
                        autoDrive8.run(Constants.AutoConstants.AutoPoints.autoEight, 0.25, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive8.isFinished() && getRuntime() > timestamp + 3) {
                            autoDrive8 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 9:
                    if (autoDrive9 == null) {
                        autoDrive9 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive9.init();
                    } else {
                        autoDrive9.run(Constants.AutoConstants.AutoPoints.autoNine, 0.25, 0.5, m_telemetry);
                        intakeCommand.disable();
                        timestamp = getRuntime();
                        if (autoDrive9.isFinished()) {
                            autoDrive9 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 10:
                    if (autoDrive10 == null) {
                        autoDrive10 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive10.init();
                    } else {
                        autoDrive10.run(Constants.AutoConstants.AutoPoints.autoTen, 0.25, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive10.isFinished()) {
                            autoDrive10 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 11:
                    if (autoDrive11 == null) {
                        autoDrive11 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive11.init();
                    } else {
                        autoDrive11.run(Constants.AutoConstants.AutoPoints.autoEleven, 0.25, 0.5, m_telemetry);
                        intakeCommand.disable();
                        timestamp = getRuntime();
                        if (autoDrive11.isFinished()) {
                            autoDrive11 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 12:
                    if (autoDrive12 == null) {
                        autoDrive12 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive12.init();
                    } else {
                        autoDrive12.run(Constants.AutoConstants.AutoPoints.autoTwelve, 0.25, 0.5, m_telemetry);
                        intakeCommand.enable();
                        if (autoDrive12.isFinished() && getRuntime() > timestamp + 3) {
                            autoDrive12 = null;
                            phase++;
                            break;
                        }
                    }
                    break;
                case 13:
                    s_drivetrain.stop();
                    intakeCommand.disable();
                    break;
            }

            m_telemetry.addData("AUTO PHASE:", phase);
            m_telemetry.update();
        }
    }
}