package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret extends SubsystemBase {

    private final DcMotor turretMotor;

    private final  PIDController turretController;

    private double setpoint;

    public Turret(HardwareMap hardwaremap) {
        turretMotor = hardwaremap.get(DcMotor.class, Constants.shooterConstants.shooterMotor);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretController = new PIDController(
                Constants.turretConstants.turretPID.turretkP,
                Constants.turretConstants.turretPID.turretkI,
                Constants.turretConstants.turretPID.turretkD);
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
            turretController.calculate(getTurretAngle(), desiredAngle);
        } else {
            turretController.calculate(getTurretAngle(), 0);
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
