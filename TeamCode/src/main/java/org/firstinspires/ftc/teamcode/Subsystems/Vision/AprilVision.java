package org.firstinspires.ftc.teamcode.Subsystems.Vision;

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
import org.firstinspires.ftc.teamcode.VisionStates;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
public class AprilVision extends SubsystemBase {

    private final MultipleTelemetry telemetry;
    private int[] desiredTagID;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private CameraStreamProcessor s_Processor;
    private final VisionStates visionStates;
    public AprilTagPoseFtc ftcPose;
    public static AprilTagDetection desiredTag;


    public static double targetRange;
    public static double targetYaw;
    public static double targetBearing;
    public static double targetY;
    public static double targetX;
    //Fx/Fy = 946.233
    //Cx = 667.521
    //Cy = 464.348
//    Radial distortion (Brown's Model)
//            K1: 0.0607443 K2: 0.0624121 K3: -0.303675
//            P1: 0.0142314 P2: 0.00530697
//            Skew: 0
    //Mean Square Reprojection Error: 0.433367 pixels

    public AprilVision(HardwareMap hardwaremap, MultipleTelemetry telemetry, VisionStates visionState) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(946.233, 946.233, 667.521, 464.348)
                .build();
        if (Constants.toggles.toggleCamStream) {
            s_Processor = new CameraStreamProcessor();
            visionPortal = new VisionPortal.Builder()
                    .addProcessor(aprilTag)
                    .addProcessor(s_Processor)
                    .setCameraResolution(new Size(1280, 800))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setCamera(hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam))
                    .build();
            FtcDashboard.getInstance().startCameraStream(s_Processor, 120);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam), aprilTag);
        }

        this.telemetry = telemetry;
        this.visionStates = visionState;
        refreshDesiredID();
    }


    public void refreshDesiredID() {
        if (visionStates.getState() == VisionStates.VisionState.MOTIF) {
            desiredTagID = new int[] {21,22,23};
        } else if (visionStates.getState() == VisionStates.VisionState.SHOOT) {
            int shootID = Constants.toggles.blueTeam ? 20 : 24;
            desiredTagID = new int[] {shootID};
        }
    }

    public boolean checkDesiredTagID(int tagID) {
        for (int i : desiredTagID) {
            if (tagID == i) {
                return true;
            }
        }
        return false;
    }

    public boolean foundTarget() {
        boolean targetFound = false;
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                refreshDesiredID();
                if (checkDesiredTagID(detection.id)) {
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
        return targetFound;
    }

    public void getAprilTagData(MultipleTelemetry m_telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        m_telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                m_telemetry.addData("Tag ID: ", detection.id);
                refreshDesiredID();
                if (checkDesiredTagID(detection.id)) {
                    desiredTag = detection;

                    String[] keys = {" Tag X", "Tag Y", "Tag Yaw", "Tag Range", "Tag Bearing"};
                    double[] tagInfo = {detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw,
                            detection.ftcPose.range, detection.ftcPose.bearing};

                    for (int x = 0; x < keys.length; x++) {
                        m_telemetry.addData(keys[x], tagInfo[x]);
                    }
                }

            }
        }
    }

    public void setVisionState(VisionStates.VisionState state) {
        visionStates.setState(state);
    }
    public void setTargetYaw(double yaw) {
        targetYaw = yaw;
    }
    public double getTargetYaw() {
        return desiredTag.ftcPose.yaw;
    }

    public void setTargetBearing(double bearing) {
        targetBearing = bearing;
    }
    public double getTargetBearing() {
        return desiredTag.ftcPose.bearing;
    }

    public void setTargetRange(double range) {
        targetRange = range;
    }
    public double getTargetRange() {
        return desiredTag.ftcPose.range;
    }

    public void setTargetY(double y) {
        targetYaw = y;
    }
    public double getTargetY() {
        return desiredTag.ftcPose.y;
    }

    public void setTargetX(double x) {
        targetX = x;
    }
    public double getTargetX() {
        return desiredTag.ftcPose.x;
    }
//
//    public void setRobotRange(double range) {
//        robotRange = range;
//    }
//    public static double getRobotRange() {
//        return robotRange;
//    }
    public void periodic() {
            refreshDesiredID();
            getAprilTagData(telemetry);
    }
}
