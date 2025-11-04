package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static java.lang.Math.min;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Shooter extends SubsystemBase {

    private final DcMotor shooterMotor;

    private final PIDController shooterController;

    private double currentSpeed;
    private double setpoint;
    public Shooter(HardwareMap hardwaremap) {
        shooterMotor = hardwaremap.get(DcMotor.class, Constants.shooterConstants.shooterMotor);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterController = new PIDController(
                Constants.shooterConstants.shooterPID.shooterkP,
                Constants.shooterConstants.shooterPID.shooterkI,
                Constants.shooterConstants.shooterPID.shooterkD);
    }

    public double getPower() {
        return shooterMotor.getPower();
    }

    public void runShooter(double desiredSpeed) {
        currentSpeed = getPower();
        shooterMotor.setPower(Math.min(shooterController.calculate(currentSpeed, desiredSpeed), Constants.shooterConstants.maxSpeed));
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

    @Override
    public void periodic() {
        runShooter(setpoint);
    }
}
