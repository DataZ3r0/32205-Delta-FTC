package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class NewTurret extends SubsystemBase {
    private final DcMotor turretMotor;
    private final PIDController turretController;
    private final MultipleTelemetry m_telemetry;
    private final Drivetrain s_drivetrain;
    private double setpoint;
    public NewTurret(HardwareMap hardwaremap, Drivetrain s_drivetrain, MultipleTelemetry m_telemetry) {
        turretMotor = hardwaremap.get(DcMotor.class, Constants.turretConstants.turretMotor);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDController(
                Constants.turretConstants.turretConfigs.turretkP,
                Constants.turretConstants.turretConfigs.turretkI,
                Constants.turretConstants.turretConfigs.turretkD);
        turretController.disableContinuousInput();
        turretController.setTolerance(1);
        this.m_telemetry = m_telemetry;
        this.s_drivetrain = s_drivetrain;

    }

    //Takes field relative input
    //Converts to robot relative
    //Compares with robot relative angle of turret
    //Applies PID for desired angle, with safeties being robot relative

    public double toRobotRelative(double value) {
        double unsafeSetpoint  = wrapAngle(value + s_drivetrain.getHeading());
        return Math.max(-90, Math.min(unsafeSetpoint, 85));
    }

    public void setSetpoint(double newSetpoint) {
        setpoint = newSetpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(setpoint - getRobotTurretAngle()) < 2;
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getRobotTurretAngle() {
        double rotations = getTurretPosition()/(28*19.2032085561 *5); //28 TPR * 19.2 motor * GR 5 belt * GR
        return rotations * 360;
    }

    public double getFieldTurretAngle() {
        return wrapAngle(getRobotTurretAngle() - s_drivetrain.getHeading());
    }

    public void setTurretPosition(double desiredAngle) {
        if (!atSetpoint()) {
            turretMotor.setPower(turretController.calculate(wrapAngle(getRobotTurretAngle()), desiredAngle));
        } else {
            stopTurret();
        }

    }

    public void stopTurret() {
        turretMotor.setPower(0);
    }

    public void periodic() {
        setTurretPosition(toRobotRelative(setpoint));

        m_telemetry.addData("robot rel turret angle", wrapAngle(getRobotTurretAngle()));
        m_telemetry.addData("field rel turret angle", wrapAngle(getFieldTurretAngle()));
        m_telemetry.addData("turret setpoint", getSetpoint());

        m_telemetry.addData("turretRelSetpoint", toRobotRelative(setpoint));
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
