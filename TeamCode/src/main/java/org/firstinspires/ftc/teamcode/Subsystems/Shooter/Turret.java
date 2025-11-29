package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Turret extends SubsystemBase {

    private final DcMotor turretMotor;
    private boolean isSafe;

    private final  PIDController turretController;
    private final MultipleTelemetry m_telemetry;

    private final Drivetrain s_drivetrain;
    private double setpoint;
    private double turretAngleTan;
    private double turretRelSetpoint;
    private double lastSetpoint;

    private double output;

    public Turret(HardwareMap hardwaremap, Drivetrain s_drivetrain, MultipleTelemetry m_telemetry) {
        turretMotor = hardwaremap.get(DcMotor.class, Constants.turretConstants.turretMotor);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDController(
                Constants.turretConstants.turretConfigs.turretkP,
                Constants.turretConstants.turretConfigs.turretkI,
                Constants.turretConstants.turretConfigs.turretkD);
        turretController.setTolerance(0.5);
//        turretController.enableContinuousInput(-180, 180);
        this.m_telemetry = m_telemetry;
        this.s_drivetrain = s_drivetrain;
    }

    public void manuelTurret(double rightStickX, double rightStickY) {
        turretAngleTan = getJoystickAngle(rightStickX, rightStickY);
        setSetpoint(wrapAngle(turretAngleTan) + s_drivetrain.getHeading());
//        setTurretAngle(turretAngleTan);
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getRawTurretAngle() {
        double revI = getTurretPosition()/(28*19.2032085561 *5); //28 TPR * 19.2 motor * GR 5 belt * GR
        return revI * 360;
    }

    public double getFieldTurretAngle() {
        double revI = getTurretPosition()/(28*19.2032085561*5); //28 TPR * 19.2 motor * GR 5 belt * GR
        double revF = revI * 360;
        return wrapAngle(revF - s_drivetrain.getHeading());
//        return wrapAngle(revF);
    }
    public double getRobotTurretAngle() {
        double revI = getTurretPosition()/(28*19.2032085561*5); //28 TPR * 19.2 motor * GR 5 belt * GR
        double revF = revI * 360;
        return wrapAngle(revF);
    }

    public double getJoystickAngle(double rightStickX, double rightStickY) {
        return wrapAngle(Math.toDegrees(Math.atan2(rightStickY, rightStickX)) + 90);
    }

    // if the turret has a setpoint within the given range of 270 (135 to -135) then turn the turret to the setpoint
    // else turn the turret to 0 degrees
    public void setTurretAngle(double desiredAngle) {

            if (Math.abs(desiredAngle) > Math.abs(90.1) || Math.abs(desiredAngle) == 180 || Math.abs(getRobotTurretAngle()) > 90.1) {
                output = Math.max(Math.min(turretController.calculate(getRobotTurretAngle(), 0), Constants.turretConstants.turretConfigs.maxSpeed), -Constants.turretConstants.turretConfigs.maxSpeed);
            } else {
                turretRelSetpoint = Math.max(Math.min(wrapAngle(desiredAngle - s_drivetrain.getHeading()), 90 - s_drivetrain.getHeading()), -90 - s_drivetrain.getHeading());
                output = Math.max(Math.min(turretController.calculate(getFieldTurretAngle(), turretRelSetpoint), Constants.turretConstants.turretConfigs.maxSpeed), -Constants.turretConstants.turretConfigs.maxSpeed);
            }


        turretMotor.setPower(output);
//        if (Math.abs(s_drivetrain.getHeading() - getFieldTurretAngle()) > 90 && !s_drivetrain.getRotatingDirection()) {
//            turretMotor.setPower(-output);
//        } else {
//
//        }
//        if (Math.abs(desiredAngle) < 135) {
//        }
//        } else {
//            turretMotor.setPower(Math.max(Math.min(turretController.calculate(wrapAngle(getTurretAngle()), wrapAngle(0)),
//                            Constants.turretConstants.turretConfigs.maxSpeed),
//                    -Constants.turretConstants.turretConfigs.maxSpeed));
//
//        turretMotor.setPower(Math.max(Math.min(turretController.calculate(wrapAngle(getTurretAngle()), wrapAngle(desiredAngle + s_drivetrain.getHeading())),
//                        Constants.turretConstants.turretConfigs.maxSpeed),
//                -Constants.turretConstants.turretConfigs.maxSpeed));



//            if(getTurretAngle() > -180) {
//                turretMotor.setPower(Math.max(Math.min(turretController.calculate(wrapAngle(getTurretAngle()), wrapAngle(-135)),
//                                Constants.turretConstants.turretConfigs.maxSpeed),
//                        -Constants.turretConstants.turretConfigs.maxSpeed));
//            } else if(getTurretAngle() < 180) {
//                turretMotor.setPower(Math.max(Math.min(turretController.calculate(wrapAngle(getTurretAngle()), wrapAngle(135)),
//                                Constants.turretConstants.turretConfigs.maxSpeed),
//                        -Constants.turretConstants.turretConfigs.maxSpeed));
//           }
//            isSafe = false;
//        }
//        } else {
//            turretMotor.setPower(Math.max(Math.min(turretController.calculate(wrapAngle(getTurretAngle()), wrapAngle(-135)),
//                            Constants.turretConstants.turretConfigs.maxSpeed),
//                    -Constants.turretConstants.turretConfigs.maxSpeed));
//        }

//        }
//        if (getTurretAngle() < Math.abs(135)) {

//        } else {
//                    turretMotor.setPower(Math.max(Math.min(turretController.calculate(getTurretAngle(), 0),
//                            Constants.turretConstants.turretConfigs.maxSpeed),
//                    -Constants.turretConstants.turretConfigs.maxSpeed));
//        }

    }

    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return turretController.atSetpoint();
    }

    public void stopTurret() {
        turretMotor.setPower(0);
    }

    public void periodic() {
        m_telemetry.addData("joysitck angle", turretAngleTan);
        m_telemetry.addData("is Safe?: ", isSafe);
        m_telemetry.addData("turret encoder position", getTurretPosition());
        m_telemetry.addData("robot rel turret angle", wrapAngle(getRobotTurretAngle()));
        m_telemetry.addData("field rel turret angle", wrapAngle(getFieldTurretAngle()));
        m_telemetry.addData("turret setpoint", getSetpoint());
        m_telemetry.addData("turretPower: ", turretMotor.getPower());
        m_telemetry.addData("pid error", turretController);
        m_telemetry.addData("at setpoint?", atSetpoint());
        m_telemetry.addData("turretRelSetpoint", turretRelSetpoint);
        setTurretAngle(setpoint);
//        setTurretAngle(Constants.turretConstants.turretConfigs.turretSetPoint);
    }
    public double wrapAngle(double angleDeg) {
        angleDeg = angleDeg % 360;     // keep within 0–360 or –360–0

        if (angleDeg > 180)
            angleDeg -= 360;

        if (angleDeg < -180)
            angleDeg += 360;

        return angleDeg;
    }
}
