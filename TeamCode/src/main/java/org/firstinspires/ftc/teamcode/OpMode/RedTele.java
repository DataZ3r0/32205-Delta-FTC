package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.MiddleStage;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.GlobalPoseEstimation;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;
import org.firstinspires.ftc.teamcode.VisionStates;

@TeleOp(name="DeltaRed", group="Teleop")
public class RedTele extends LinearOpMode {

    GamepadEx driverGamepad;
    GamepadEx opGamepad;
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
    GamepadKeys.Button shooterTestButtonTwo;
    boolean intakeReversed;
    boolean isTurretTurning;

    double shooterTimestamp;
    double turretTimestamp;

    double tagTurretSetpoint;

    double givenRPM;
    boolean manualShootAllowed;

    @Override
    public void runOpMode() {

        driverGamepad = new GamepadEx(gamepad1);
        opGamepad = new GamepadEx(gamepad2);

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionState = new VisionStates();

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState, 24);
        s_intake = new Intake(hardwareMap);
        s_middleStage = new MiddleStage(hardwareMap);
        s_shooter = new Shooter(hardwareMap, m_telemetry, opGamepad.isDown(GamepadKeys.Button.A));
        s_turret = new Turret(hardwareMap, s_drivetrain, m_telemetry);

        s_otos = new OTOS(hardwareMap, m_telemetry);
//        poseEstimation = new GlobalPoseEstimation(s_otos, s_aprilVision, s_turret);

        intakeTrigger = GamepadKeys.Trigger.RIGHT_TRIGGER;
        outtakeTrigger = GamepadKeys.Trigger.LEFT_TRIGGER;

        shooterButton = GamepadKeys.Button.X;
        shooterTestButtonTwo = GamepadKeys.Button.Y;

        intakeReversed = false;

        visionState.setState(VisionStates.VisionState.SHOOT);

        CommandScheduler.getInstance().run();
        s_turret.stopTurret();
        s_drivetrain.resetYaw();

        givenRPM = 2400;


        waitForStart();

        while (opModeIsActive()) {

            s_drivetrain.periodic();
            s_otos.periodic();
            s_shooter.periodic();
            s_turret.periodic();
            s_aprilVision.periodic();
//            poseEstimation.periodic();

            s_drivetrain.drive(
                    driverGamepad.getLeftY(),
                    driverGamepad.getLeftX(),
                    driverGamepad.getRightX()
            );

//            if (s_aprilVision.foundTarget()) {
//                s_otos.setPose(poseEstimation.getPose());
//            }

            driverGamepad.readButtons();
            opGamepad.readButtons();

            if(!(s_aprilVision.foundTarget()) && getRuntime() > turretTimestamp + 2) {
                    if (Math.abs(opGamepad.getRightX()) > 0.01 || Math.abs(opGamepad.getRightY()) > 0.01) {
                        s_turret.manuelTurret(
                                opGamepad.getRightX(), opGamepad.getRightY());
//                        isTurretTurning = false;
                }
            } else {
                tagTurretSetpoint = (s_turret.getRobotTurretAngle() + s_aprilVision.getTx());
                s_turret.setSetpoint(tagTurretSetpoint);
                turretTimestamp = getRuntime();


//                isTurretTurning = true;
            }
//            m_telemetry.addData("TURRET PLACEHOLDER TURN", isTurretTurning);

            if (opGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                givenRPM = 2200;
            } else if (opGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                givenRPM = 2450;
            } else if (opGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                givenRPM = 3150;
            }

            if (driverGamepad.isDown(shooterTestButtonTwo)) {
                if (s_aprilVision.foundTarget()) {
                    s_shooter.setDesiredVelocity(s_aprilVision.getRangeAvg());
                } else {
                    s_shooter.setSetpoint(givenRPM);
                }
//                s_shooter.setSetpoint(Constants.shooterConstants.shooterConfigs.testRPM);
                shooterTimestamp = getRuntime();
                if (s_shooter.atSetpoint() && !triggerDown(driverGamepad, outtakeTrigger)) {
                    s_shooter.runLoader();
                } else {
                    if (!triggerDown(driverGamepad, intakeTrigger)) {
                        s_shooter.stopLoader();
                    }
                }
            } else if (getRuntime() > shooterTimestamp + 2){
                s_shooter.setSetpoint(0);
                s_shooter.stopLoader();
            }

            if (s_aprilVision.foundTarget() || opGamepad.isDown(GamepadKeys.Button.Y)) {
                manualShootAllowed = true;
            } else {
                manualShootAllowed = false;
            }

            s_shooter.stopperPeriodic(opGamepad, GamepadKeys.Button.Y, manualShootAllowed);


            if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)){
                s_drivetrain.resetYaw();
            }

            if(!triggerDown(driverGamepad, intakeTrigger)
                    && !triggerDown(driverGamepad, outtakeTrigger)
                    && !driverGamepad.isDown(shooterTestButtonTwo)) {
                s_middleStage.stop();
                s_shooter.stopLoader();
            }

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
                s_otos.resetOTOS();
            }

            if (triggerDown(driverGamepad, intakeTrigger)) {
                s_intake.runIntake(driverGamepad.getTrigger(intakeTrigger));
                s_middleStage.runIntake(driverGamepad.getTrigger(intakeTrigger));
                s_shooter.runLoader();
            } else if (triggerDown(driverGamepad, outtakeTrigger)) {
                s_intake.runOuttake(driverGamepad.getTrigger(outtakeTrigger));
                s_shooter.outtake();
                s_middleStage.runOuttake(driverGamepad.getTrigger(outtakeTrigger));
            } else {
              s_intake.stop();
            }


            m_telemetry.addData("a pressed", driverGamepad.isDown(GamepadKeys.Button.A));
            m_telemetry.update();
        }
    }

    public boolean triggerDown(GamepadEx gamepad, GamepadKeys.Trigger trigger) {
        return gamepad.getTrigger(trigger) > 0.001;
    }
}
//merge commit
