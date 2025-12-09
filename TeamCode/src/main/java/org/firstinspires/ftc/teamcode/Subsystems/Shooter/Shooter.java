package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static java.lang.Math.min;

import android.content.res.ColorStateList;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;

public class Shooter extends SubsystemBase {

    private final DcMotorEx shooterMotor;
    private final Servo stopperServo;
    private final CRServo loadingServo;

    private final MultipleTelemetry telemetry;

    private final PIDController shooterController;
    private final SimpleMotorFeedforward shooterFeedforward;

    private double currentVelocity;
    private double setpoint;

    private double shootingCurrentThresh;

    private boolean controllerInput;

    private boolean lastState, currState;
    public Shooter(HardwareMap hardwaremap, MultipleTelemetry telemetry, boolean controllerInput) {
        shooterMotor = hardwaremap.get(DcMotorEx.class, Constants.shooterConstants.shooterMotor);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        loadingServo = hardwaremap.get(CRServo.class, Constants.shooterConstants.loadingServo);
        stopperServo = hardwaremap.get(Servo.class, "stopperServo");

        shooterController = new PIDController(
                Constants.shooterConstants.shooterConfigs.shooterkP,
                Constants.shooterConstants.shooterConfigs.shooterkI,
                Constants.shooterConstants.shooterConfigs.shooterkD);

        shooterFeedforward = new SimpleMotorFeedforward(
                Constants.shooterConstants.shooterConfigs.shooterkS,
                Constants.shooterConstants.shooterConfigs.shooterkV
        );

        this.controllerInput = controllerInput;
        this.telemetry = telemetry;
    }

    public boolean hasControllerInput() {
        return controllerInput;
    }
    public void runLoader() {
        loadingServo.setDirection(DcMotorSimple.Direction.FORWARD);
        loadingServo.setPower(1);
    }
    public void stopLoader() {
        loadingServo.setPower(0);
    }
    public void outtake() {
        loadingServo.setDirection(DcMotorSimple.Direction.REVERSE);
        loadingServo.setPower(-1);
    }

    public void openStopper() {
        stopperServo.setPosition(0.3);
    }

    public void closeStopper() {
        stopperServo.setPosition(0.1);
    }


    public void setPower(double desiredPower) {
        shooterMotor.setPower(desiredPower);
    }
    public double getPower() {
        return shooterMotor.getPower();
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getRPM() {
        return (getVelocity()/Constants.shooterConstants.ticksPerRev*60);
    }

    public void runShooter(double desiredVelocity) {
        double currentVelocity = getRPM();

        shooterMotor.setPower(Math.max(Math.min((shooterController.calculate(currentVelocity, desiredVelocity)
                        + shooterFeedforward.calculate(desiredVelocity)),
                Constants.shooterConstants.shooterConfigs.maxSpeed), 0));

    }

    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getShooterCurrent(){
        return shooterMotor.getCurrent(CurrentUnit.AMPS);
    }

//    public boolean isShooting() {
//        double shootingCurrentThresh = 3;
//        return getShooterCurrent() > shootingCurrentThresh;
//    }

//    public void readVal() {
//        lastState = currState;
//        currState = isShooting();
//    }

//    public boolean wasBallShot() {
//        return (lastState && !currState);
//    }

//-0.0116909x^{2}+10.43324x+1822.88718
    public void setDesiredVelocity(double targetRange) {
//        double tagDistanceMetres = targetRange * 0.0254;
        double desiredVelocity = (-0.0116909 * Math.pow(targetRange, 2)) + (10.43324 * targetRange) + 1822.88718;
        setSetpoint(desiredVelocity);
    }
    public boolean atSetpoint() {
        return getSetpoint() - getRPM() < Math.abs(Constants.shooterConstants.shooterRPMTolerance);
    }
    public void stop() {
        shooterMotor.setPower(0);
    }

    public void periodic() {
//        readVal();

        if (!hasControllerInput()) {
            if (atSetpoint() && getSetpoint() > 1000) {
                openStopper();
            } else {
                closeStopper();
            }
            runShooter(setpoint);
        } else {
            openStopper();
            outtake();
            shooterMotor.setPower(-1);
        }

        telemetry.addData("Shooter RPM: ", getRPM());
//        runShooter(Constants.shooterConstants.shooterConfigs.testRPM);
        telemetry.addData("Shooter Current: ", getShooterCurrent());
        telemetry.addData("shooter setpoint: ", getSetpoint());
        telemetry.addData("shooter at setpoint?", atSetpoint());
        telemetry.addData("stopper pos", stopperServo.getPosition());
    }
}
