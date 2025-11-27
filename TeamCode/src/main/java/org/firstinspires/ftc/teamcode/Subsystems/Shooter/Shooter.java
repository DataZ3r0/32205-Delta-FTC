package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static java.lang.Math.min;

import android.content.res.ColorStateList;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;

public class Shooter extends SubsystemBase {

    private final DcMotorEx shooterMotor;
//    private final ServoEx loadingServo;

    private final MultipleTelemetry telemetry;

    private final PIDController shooterController;
    private final SimpleMotorFeedforward shooterFeedforward;

    private double currentVelocity;
    private double setpoint;

    private double shootingCurrentThresh;

    private boolean lastState, currState;
    public Shooter(HardwareMap hardwaremap, MultipleTelemetry telemetry) {
        shooterMotor = hardwaremap.get(DcMotorEx.class, Constants.shooterConstants.shooterMotor);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        loadingServo = hardwaremap.get(ServoEx.class, Constants.shooterConstants.loadingServo);
//        loadingServo.setInverted(Constants.shooterConstants.loadingServoRev);

        shooterController = new PIDController(
                Constants.shooterConstants.shooterConfigs.shooterkP,
                Constants.shooterConstants.shooterConfigs.shooterkI,
                Constants.shooterConstants.shooterConfigs.shooterkD);

        shooterFeedforward = new SimpleMotorFeedforward(
                Constants.shooterConstants.shooterConfigs.shooterkS,
                Constants.shooterConstants.shooterConfigs.shooterkV
        );

        this.telemetry = telemetry;
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

//    public void runLoader() {
//        loadingServo.rotateBy(Constants.shooterConstants.loadingServoSpeed);
//    }

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

    public void setDesiredVelocity(double targetRange) {
//        double tagDistanceMetres = targetRange;
//        y=-0.00181926x^{2}+7.60048x+1849.78184
        double desiredVelocity = (-0.00181926 * Math.pow(targetRange, 2)) + (7.60048 * targetRange) + 1849.78184;
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
        runShooter(setpoint);
        telemetry.addData("Shooter RPM: ", getRPM());
        telemetry.addData("Shooter Current: ", getShooterCurrent());
        telemetry.addData("shooter setpoint: ", getSetpoint());
        telemetry.addData("shooter at setpoint?", atSetpoint());
    }
}
