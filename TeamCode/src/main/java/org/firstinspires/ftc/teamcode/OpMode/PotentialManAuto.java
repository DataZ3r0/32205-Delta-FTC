package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.AutoDrive;
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

@Autonomous(name="Delta-3Piece", group="Auto")
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

    AutoDrive autoDrive;

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

        driveController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.0, 0.0);
        rotationController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.0, 0.0);

        autoDrive = new AutoDrive(s_drivetrain, s_otos);

        CommandScheduler.getInstance().run();
        s_turret.stopTurret();
        s_drivetrain.resetYaw();
        autoDrive.init();
        s_otos.setPose(Constants.AutoConstants.AutoPoints.startpos);

        timestamp = getRuntime();
        phase = 0;
        setpoint = 0;


        waitForStart();

        while(opModeIsActive()) {
            s_aprilVision.periodic();
            s_shooter.periodic();
            s_turret.periodic();
            s_otos.periodic();

            if (s_aprilVision.foundTarget()) {
                s_turret.setSetpoint(s_turret.getRobotTurretAngle() + s_aprilVision.getTx());
                s_shooter.setDesiredVelocity(s_aprilVision.getRangeAvg());
            } else {
                s_shooter.setSetpoint(0);
                s_turret.setSetpoint(s_otos.getH());
            }

            m_telemetry.update();

            switch (phase) {
                case 0:
                    autoDrive.run(Constants.AutoConstants.AutoPoints.autoOne, 0.5, 0.2);
                    if (autoDrive.isFinished()) {
                        timestamp = getRuntime();
                        phase++;
                        break;
                    }
//                case 1:
//                    s_intake.runIntake(1);
//                    s_middleStage.runIntake(1);
//                    s_shooter.runLoader();
//                    if (getRuntime() > timestamp + 3) {
//                        autoDrive.init();
//                        phase++;
//                        break;
//                    }
//                case 2:
//                    autoDrive.run(Constants.AutoConstants.AutoPoints.autoTwo, 0.5, 0.2);
//                    if (autoDrive.isFinished()) {
//                        s_intake.stop();
//                        s_middleStage.stop();
//                        s_shooter.stopLoader();
//                        phase++;
//                        break;
//                    }
//                case 3:
//                    autoDrive.run(Constants.AutoConstants.AutoPoints.autoThree, 0.5, 0.2);
//                    if (autoDrive.isFinished()) {
//                        timestamp = getRuntime();
//                        s_intake.runIntake(1);
//                        s_middleStage.runIntake(1);
//                        s_shooter.runLoader();
//                    }
            }
        }
    }
}