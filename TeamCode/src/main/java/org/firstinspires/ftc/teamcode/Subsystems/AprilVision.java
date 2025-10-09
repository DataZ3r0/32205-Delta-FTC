package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.vision.VisionPortal.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import java.util.ArrayList;
import java.util.List;
public class AprilVision extends SubsystemBase {

    final int DESIRED_TAG_ID = -1;
    private AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final CameraStreamProcessor s_Processor;
    public AprilTagPoseFtc ftcPose;
    public static AprilTagDetection desiredTag = null;

    public static double targetRange;
    public static double targetYaw;
    public static double targetBearing;
    public static double targetY;
    public static double targetX;

    public AprilVision(HardwareMap hardwaremap) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (Constants.toggles.toggleCamStream) {
            s_Processor = new CameraStreamProcessor();
            visionPortal = new VisionPortal.Builder()
                    .addProcessor(aprilTag)
                    .addProcessor(s_Processor)
                    .setCamera(hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam))
                    .build();
            FtcDashboard.getInstance().startCameraStream(s_Processor, 60);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam), aprilTag);
        }
    }

    public void getAprilTagData(MultipleTelemetry m_telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        m_telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                m_telemetry.addData("Tag ID: ", detection.id);

                String[] keys = {" Tag X", "Tag Y", "Tag Yaw", "Tag Range", "Tag Bearing"};
                double[] tagInfo = {detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw,
                        detection.ftcPose.range, detection.ftcPose.bearing};

                for (int x = 0; x < 5; x++) {
                    m_telemetry.addData(keys[x], tagInfo[x]);
                }
            }
        }
    }

    public boolean findTarget() {
        boolean targetFound = false;
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
            }
        }
        return findTarget();
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
