package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

@Autonomous(name="testautofunk", group="Auto")
public class testautofunkiness extends LinearOpMode {
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

    @Override
    public void runOpMode() {

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionState = new VisionStates();

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState,20);
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
//        s_drivetrain.resetYaw();
        intakeCommand.disable();
//        autoDrive.init();
//        s_otos.setPose(Constants.AutoConstants.AutoPoints.startpos);
//
        timestamp = getRuntime();
        phase = 0;
//        setpoint = 0;


        waitForStart();

        while(opModeIsActive()) {
            s_drivetrain.periodic();
//            s_aprilVision.periodic();
//            s_shooter.periodic();
//            s_turret.periodic();
            s_otos.periodic();

            intakeCommand.periodic();

//            if (s_aprilVision.foundTarget()) {
//                s_turret.setSetpoint(s_turret.getRobotTurretAngle() + s_aprilVision.getTx());
//                s_shooter.setDesiredVelocity(s_aprilVision.getRangeAvg());
//            } else {
//                s_shooter.setSetpoint(0);
//                s_turret.setSetpoint(s_drivetrain.getHeading());
//            }

//            s_shooter.stopperPeriodic(null, null);

            switch (phase) {
                case 0:
                    if (autoDrive1 == null) {
                        autoDrive1 = new AutoDrive(s_drivetrain,s_otos);
                        autoDrive1.init();
                    } else {
                        autoDrive1.run(Constants.AutoConstants.AutoPoints.autoOne, 1.0, 1.0, m_telemetry);
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
                        autoDrive2.run(Constants.AutoConstants.AutoPoints.autoTwo, 1.0, 1.0, m_telemetry);
                        if (autoDrive2.isFinished()) {
                            autoDrive2 = null;
                            phase = 0;
                            break;
                        }
                    }
                    break;
            }

            m_telemetry.addData("AUTO PHASE:", phase);
            m_telemetry.update();
        }
    }
}