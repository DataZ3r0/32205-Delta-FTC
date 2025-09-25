package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.vision.VisionPortal.*;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
public class AprilVision extends SubsystemBase {

    final int DESIRED_TAG_ID = -1;
    final AprilTagProcessor aprilTag;
    final VisionPortal visionPortal;
    public AprilTagPoseFtc ftcPose;
    public static AprilTagDetection desiredTag = null;

    public static double targetRange;
    public static double targetYaw;
    public static double targetBearing;
    public static double targetY;
    public static double targetX;

    public AprilVision(HardwareMap hardwaremap) {
        final boolean USE_WEBCAM = true;

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam), aprilTag);
    }

    public void getAprilTagData(Telemetry telemetry) {
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("Robot XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Robot PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
                telemetry.addLine(String.format("Tag XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Tag PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Tag RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                setTargetYaw(detection.ftcPose.yaw);
                setTargetRange(detection.ftcPose.range);
                setTargetBearing(detection.ftcPose.bearing);
                setTargetY(detection.ftcPose.y);
                setTargetX(detection.ftcPose.x);
//                setTargetBearing(detection.ftcPose.bearing);

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    public boolean findTarget() {
        boolean targetFound = false;
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return targetFound;
    }

    public void setTargetYaw(double yaw) {
        targetYaw = yaw;
    }
    public static double getTargetYaw() {
        return targetYaw;
    }

    public void setTargetBearing(double bearing) {
        targetBearing = bearing;
    }
    public static double getTargetBearing() {
        return targetBearing;
    }

    public void setTargetRange(double range) {
        targetRange = range;
    }
    public static double getTargetRange() {
        return targetRange;
    }

    public void setTargetY(double y) {
        targetYaw = y;
    }
    public static double getTargetY() {
        return targetY;
    }

    public void setTargetX(double x) {
        targetX = x;
    }
    public static double getTargetX() {
        return targetX;
    }
//
//    public void setRobotRange(double range) {
//        robotRange = range;
//    }
//    public static double geRobotRange() {
//        return robotRange;
//    }
    @Override
    public void periodic() {

    }
}
