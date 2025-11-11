package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static java.lang.Math.min;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Constants;

public class Shooter extends SubsystemBase {

    private final DcMotorEx shooterMotor;

    private MultipleTelemetry telemetry;

    private final PIDController shooterController;

    private double currentVelocity;
    private double setpoint;
    public Shooter(HardwareMap hardwaremap, MultipleTelemetry telemetry) {
        shooterMotor = hardwaremap.get(DcMotorEx.class, Constants.shooterConstants.shooterMotor);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterController = new PIDController(
                Constants.shooterConstants.shooterPID.shooterkP,
                Constants.shooterConstants.shooterPID.shooterkI,
                Constants.shooterConstants.shooterPID.shooterkD);

        this.telemetry = telemetry;
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

    public void runShooter(double desiredSpeed) {
//        currentVelocity = getRPM();
//        shooterMotor.setVelocity(Math.min(shooterController.calculate(currentVelocity, desiredVelocity), Constants.shooterConstants.shooterPID.maxSpeed));
        shooterMotor.setPower(desiredSpeed);
    }

    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return getSetpoint() - getPower() < Math.abs(0.01);
    }
    public void stop() {
        shooterMotor.setPower(0);
    }

    public void periodic() {
        if (setpoint > 0.01) {
            runShooter(setpoint);
        } else {
            stop();
        }

        telemetry.addData("Shooter RPM: ", getRPM());
    }
}
