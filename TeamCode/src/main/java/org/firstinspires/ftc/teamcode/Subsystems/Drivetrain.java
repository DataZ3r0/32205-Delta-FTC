package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;

public class Drivetrain extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private final BNO055IMU IMU;

    private double yawOffset;
    private double turnSpeed;
    private double headingError;
    private double headingThreshold;
    private double driveSpeed;
    private double strafeSpeed;

    public Drivetrain(HardwareMap hardwaremap, Boolean autoMode) {
        frontLeft = hardwaremap.get(DcMotor.class, Constants.DrivetrainConstants.frontLeftMotor);
        frontRight = hardwaremap.get(DcMotor.class, Constants.DrivetrainConstants.frontRightMotor);
        backLeft = hardwaremap.get(DcMotor.class, Constants.DrivetrainConstants.backLeftMotor);
        backRight = hardwaremap.get(DcMotor.class, Constants.DrivetrainConstants.backRightMotor);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        if(autoMode) {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        IMU = hardwaremap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        yawOffset = IMU.getAngularOrientation().firstAngle - Constants.DrivetrainConstants.controlHubOffset;
    }

    public void drive(double driveX, double driveY, double rotation) {

        double botHeading = getHeading();
        double headingRadians = Math.toRadians(botHeading);

        double sin =  Math.sin(-headingRadians);
        double cos =  Math.cos(-headingRadians);

        double fieldOrientedX = driveX * cos - driveY * sin;
        double fieldOrientedY = driveX * sin + driveY * cos;

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
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Drive train");
        telemetry.addData("Heading: ", getHeading());

        telemetry.addData("Front Left Power: ", frontLeft.getPower());

        telemetry.addData("Front Right Power: ", frontRight.getPower());

        telemetry.addData("Back Left Power: ", backLeft.getPower());

        telemetry.addData("Back Right Power: ", backRight.getPower());
    }

    public void StraightForward(double maxSpeed, double distance, double heading) {
        int moveCounts = (int)(distance * Constants.DrivetrainConstants.CountsPerInch);
        int frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
        int frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
        int backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
        int backRightTarget = backRight.getCurrentPosition() + moveCounts;

        frontLeft.setTargetPosition(-frontLeftTarget);
        frontRight.setTargetPosition(-frontRightTarget);
        backLeft.setTargetPosition(-backLeftTarget);
        backRight.setTargetPosition(-backRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxSpeed = Math.abs(maxSpeed);
        drive(maxSpeed, 0, 0);
        while ((frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            drive(driveSpeed, strafeSpeed, turnSpeed);
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        drive(0, 0, 0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Strafe(double maxSpeed, double distance, double heading) {
        int moveCounts = (int)(distance * Constants.DrivetrainConstants.CountsPerInch);

    }
//    public void turnToHeading(double heading) {
//
//        // Run getSteeringCorrection() once to pre-calculate the current error
//        getSteeringCorrection(heading);
//
//        // keep looping while we are still active, and not on heading.
//        while ((Math.abs(headingError) > headingThreshold)) {
//
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading);
//
//            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -Constants.DrivetrainConstants.maxDrive,
//                    Constants.DrivetrainConstants.maxDrive);
//
//            // Pivot in place by applying the turning correction
//            drive(0, 0, turnSpeed);
//        }

        // Stop all motion;
//        drive(0, 0, 0);
//    }
//
//
    public double getSteeringCorrection(double desiredHeading) {
        headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * 0.02,
                -Constants.DrivetrainConstants.maxDrive,
                Constants.DrivetrainConstants.maxDrive);
    }
}
