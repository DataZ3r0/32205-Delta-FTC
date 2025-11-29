package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name="Delta-Farback3Piece", group="Auto")
public class FarBack3Piece extends LinearOpMode {
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

    double setpoint, rotsetpoint;
    double timestamp;
    double output;

    PIDController driveController;

    @Override
    public void runOpMode() {

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionState = new VisionStates();

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState);
        s_intake = new Intake(hardwareMap);
        s_middleStage = new MiddleStage(hardwareMap);
        s_shooter = new Shooter(hardwareMap, m_telemetry);
//        s_turret = new Turret(hardwareMap, s_drivetrain, m_telemetry);

        s_otos = new OTOS(hardwareMap, m_telemetry);
//        poseEstimation = new GlobalPoseEstimation(s_otos, s_aprilVision, s_turret);

        intakeReversed = false;

        visionState.setState(VisionStates.VisionState.SHOOT);

        driveController = new PIDController(Constants.DrivetrainConstants.drivePID.kPdrive, 0.0, 0.0);

        CommandScheduler.getInstance().run();
        s_turret.stopTurret();
        s_drivetrain.resetYaw();

        phase = 0;
        setpoint = Constants.toggles.blueTeam ? -30 : 30;
        rotsetpoint = Constants.toggles.blueTeam ? 90 : -90;

        timestamp = getRuntime();


        waitForStart();

        while(opModeIsActive()) {
            s_aprilVision.periodic();
            s_shooter.periodic();
//            s_turret.periodic();
            s_otos.periodic();

            if (s_aprilVision.foundTarget()) {
                s_turret.setSetpoint(s_turret.getRobotTurretAngle() + s_aprilVision.getTx());
            }

            switch (phase) {
                case 0:
                    s_shooter.setSetpoint(3200);
                    s_intake.runIntake(1);
                    s_middleStage.runIntake(1);
                    if(s_shooter.atSetpoint()) {
                        s_shooter.runLoader();
                    }
                    if (getRuntime() > timestamp + 15) {
                        s_shooter.setSetpoint(0);
                        phase++;
                    }
                case 1:
                    output = driveController.calculate(s_otos.getX(), setpoint);
                    s_drivetrain.drive(0, output, 0);
                    if (setpoint - s_otos.getY() < 4) {
                        s_drivetrain.stop();
                        phase++;
                        break;
                    }
                case 2:
                    output = driveController.calculate(s_otos.getH(), rotsetpoint);
                    s_drivetrain.drive(0,0, output);
                    if (setpoint - s_otos.getH() < 4) {
                        s_drivetrain.stop();
                        phase++;
                        break;
                    }
            }
        }
    }
}