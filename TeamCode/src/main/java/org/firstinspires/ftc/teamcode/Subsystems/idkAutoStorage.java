//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.Constants;
//
//public class idkAutoStorage {
//if(autoMode) {
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        } else {
//                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

//    public void StraightForward(double maxSpeed, double distance, double heading) {
//        int moveCounts = (int)(distance * Constants.DrivetrainConstants.CountsPerInch);
//        int frontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
//        int frontRightTarget = frontRight.getCurrentPosition() + moveCounts;
//        int backLeftTarget = backLeft.getCurrentPosition() + moveCounts;
//        int backRightTarget = backRight.getCurrentPosition() + moveCounts;
//
//        frontLeft.setTargetPosition(-frontLeftTarget);
//        frontRight.setTargetPosition(-frontRightTarget);
//        backLeft.setTargetPosition(-backLeftTarget);
//        backRight.setTargetPosition(-backRightTarget);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        maxSpeed = Math.abs(maxSpeed);
//        drive(maxSpeed, 0, 0);
//        while ((frontLeft.isBusy() && frontRight.isBusy()
//                && backLeft.isBusy() && backRight.isBusy())) {
//
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading);
//
//            // if driving in reverse, the motor correction also needs to be reversed
//            if (distance < 0)
//                turnSpeed *= -1.0;
//
//            // Apply the turning correction to the current driving speed.
//            drive(driveSpeed, strafeSpeed, turnSpeed);
//        }
//
//        // Stop all motion & Turn off RUN_TO_POSITION
//        drive(0, 0, 0);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void Strafe(double maxSpeed, double distance, double heading) {
//        int moveCounts = (int)(distance * Constants.DrivetrainConstants.CountsPerInch);
//
//    }
////    public void turnToHeading(double heading) {
////
////        // Run getSteeringCorrection() once to pre-calculate the current error
////        getSteeringCorrection(heading);
////
////        // keep looping while we are still active, and not on heading.
////        while ((Math.abs(headingError) > headingThreshold)) {
////
////            // Determine required steering to keep on heading
////            turnSpeed = getSteeringCorrection(heading);
////
////            // Clip the speed to the maximum permitted value.
////            turnSpeed = Range.clip(turnSpeed, -Constants.DrivetrainConstants.maxDrive,
////                    Constants.DrivetrainConstants.maxDrive);
////
////            // Pivot in place by applying the turning correction
////            drive(0, 0, turnSpeed);
////        }
//
//    // Stop all motion;
////        drive(0, 0, 0);
////    }
////
////
//    public double getSteeringCorrection(double desiredHeading) {
//        headingError = desiredHeading - getHeading();
//
//        // Normalize the error to be within +/- 180 degrees
//        while (headingError > 180)  headingError -= 360;
//        while (headingError <= -180) headingError += 360;
//
//        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//        return Range.clip(headingError * 0.02,
//                -Constants.DrivetrainConstants.maxDrive,
//                Constants.DrivetrainConstants.maxDrive);
//    }
//}
