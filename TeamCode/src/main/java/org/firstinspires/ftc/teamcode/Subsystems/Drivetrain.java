package org.firstinspires.ftc.teamcode.Subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;

public class Drivetrain extends SubsystemBase {

    private final MultipleTelemetry telemetry;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private final BNO055IMU IMU;

    private final PIDController drivePID = new PIDController(
            Constants.DrivetrainConstants.drivePID.kPdrive
            , 0.0
            , 0.0);
    private final PIDController strafePID = new PIDController(
            Constants.DrivetrainConstants.drivePID.kPstrafe
            , 0.0
            , 0.0);
    private final PIDController turnPID = new PIDController(
            Constants.DrivetrainConstants.drivePID.kPturn
            , 0.0
            , 0.0);


    private double yawOffset;
    private double azimuth;
    public Drivetrain(HardwareMap hardwaremap, MultipleTelemetry telemetry) {
        frontLeft = hardwaremap.get(DcMotorEx.class, Constants.DrivetrainConstants.frontLeftMotor);
        frontRight = hardwaremap.get(DcMotorEx.class, Constants.DrivetrainConstants.frontRightMotor);
        backLeft = hardwaremap.get(DcMotorEx.class, Constants.DrivetrainConstants.backLeftMotor);
        backRight = hardwaremap.get(DcMotorEx.class, Constants.DrivetrainConstants.backRightMotor);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU = hardwaremap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        this.telemetry = telemetry;
    }

    public void drive(double driveY, double driveX, double rotation) {

        double botHeading = getHeading();
        double headingRadians = Math.toRadians(botHeading);

        double sin =  Math.sin(-headingRadians);
        double cos =  Math.cos(-headingRadians);

        double fieldOrientedX = driveY * cos - driveX * sin;
        double fieldOrientedY = driveY * sin + driveX * cos;

        fieldOrientedX *= Constants.DrivetrainConstants.strafingBalancer;

        double denominator = Math.max(Math.abs(fieldOrientedY) + Math.abs(fieldOrientedX) + Math.abs(rotation), 1);

        double frontLeftPower = (fieldOrientedY + fieldOrientedX + rotation) / denominator;
        double frontRightPower = (fieldOrientedY - fieldOrientedX - rotation) / denominator;
        double backLeftPower = (fieldOrientedY - fieldOrientedX + rotation) / denominator;
        double backRightPower = (fieldOrientedY + fieldOrientedX - rotation) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public double getRawHeading() {
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public double getHeading() {
        double heading = getRawHeading() - yawOffset;

        if(heading > 180) {
            heading -= 360;
        } else if (heading < -180) {
            heading += 360;
        }

        return heading;
    }

    public void resetYaw() {
        yawOffset = getRawHeading() - Constants.DrivetrainConstants.controlHubOffset;
    }

    public void stop(OTOS otos) {
//        double yPow = drivePID.calculate(otos.getY(), otos.getY());
//        double xPow = strafePID.calculate(otos.getX(), otos.getX());
//        double hPow = turnPID.calculate(otos.getH(), otos.getH());

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

//        drive(yPow, xPow, hPow);
    }

    public void resetAzimuth() {
        azimuth = getHeading();
    }

    public void periodic() {
        telemetry.addData("DRIVE: Heading: ", getHeading());
//        m_telemetry.addData("DRIVE: Front Left Power: ", frontLeft.getPower());
//        m_telemetry.addData("DRIVE: Front Right Power: ", frontRight.getPower());
//        m_telemetry.addData("DRIVE: Back Left Power: ", backLeft.getPower());
//        m_telemetry.addData("DRIVE: Back Right Power: ", backRight.getPower());
    }


}
