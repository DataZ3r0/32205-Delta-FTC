package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EE_Ctrl;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.features2d.SimpleBlobDetector_Params;

import java.util.HashMap;
import java.util.List;

public class AprilTagDetection {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;
    private Telemetry dashboardTelemetry;
    private CameraStreamProcessor s_Processor;

    public AprilTagDetection(HardwareMap hardwaremap) {
        s_Processor = new CameraStreamProcessor();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam), aprilTag);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .addProcessor(s_Processor)
                .setCamera(hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam))
                .build();
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        FtcDashboard.getInstance().startCameraStream(s_Processor, 60);
    }

    public void getAprilTagData(Telemetry telemetry) {
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (Constants.compMode) {
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            // Step through the list of detections and display info for each one.
            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        } else {
            dashboardTelemetry.addData("# AprilTags Detected", currentDetections.size());
            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    dashboardTelemetry.addData("Tag ID: ", detection.id);

                    String[][] keys = {{"X", "Y", "Z"}, {"Pitch", "Roll", "Yaw"}, {"Range", "Bearing", "Elevation"}};
                    double[][] tagInfo = {{detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z},
                            {detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw},
                            {detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation}};

                    for (int x = 0; x < 3; x++) {
                        for (int y = 0; y < 3; y++) {
                            dashboardTelemetry.addData(keys[x][y], tagInfo[x][y]);
                        }
                    }
                } else {
                    dashboardTelemetry.addData("No Tag Detected", null);
                }
                dashboardTelemetry.update();
            }
        }
    }
}
