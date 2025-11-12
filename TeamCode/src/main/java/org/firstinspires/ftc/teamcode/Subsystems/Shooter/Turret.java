package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret extends SubsystemBase {

    private final DcMotorEx turretMotor;

    private final  PIDController turretController;

    private double setpoint;

    public Turret(HardwareMap hardwaremap) {
        turretMotor = hardwaremap.get(DcMotorEx.class, Constants.shooterConstants.shooterMotor);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretController = new PIDController(
                Constants.turretConstants.turretConfigs.turretkP,
                Constants.turretConstants.turretConfigs.turretkI,
                Constants.turretConstants.turretConfigs.turretkD);
    }

    public void manuelTurret(double rightStickX, double rightStickY) {
        double ratio = rightStickX / rightStickY;
        setTurretAngle(ratio);
    }

    public double getTurretPosition() {
        return (double) turretMotor.getCurrentPosition();
    }

    public double getTurretAngle() {
        double rev = getTurretPosition()/(1464*5);
        return rev * 360;
    }

    public void setTurretAngle(double desiredAngle) {
        if (getTurretAngle() < Math.abs(135)) {
            turretMotor.setPower(Math.max(Math.min(turretController.calculate(getTurretAngle(), desiredAngle),
                            Constants.turretConstants.turretConfigs.maxSpeed),
                            -Constants.turretConstants.turretConfigs.maxSpeed));
        } else {
            turretMotor.setPower(Math.max(Math.min(turretController.calculate(getTurretAngle(), 0),
                            Constants.turretConstants.turretConfigs.maxSpeed),
                            -Constants.turretConstants.turretConfigs.maxSpeed));
        }

    }

    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return getSetpoint() - getTurretAngle() < Math.abs(2);
    }

    public void periodic() {
        setTurretAngle(setpoint);
    }
}
