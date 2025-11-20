package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.vision.VisionPortal.*;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
//    private final AprilTagProcessor aprilTag;
//    private final VisionPortal visionPortal;
//    private final CameraStreamProcessor s_Processor;
    private final VisionStates visionStates;
    public AprilTagPoseFtc ftcPose;
    public LLResultTypes.FiducialResult desiredTag;

    private Limelight3A limelight;


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
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setLensIntrinsics(946.233, 946.233, 667.521, 464.348)
//                .build();
//        if (Constants.toggles.toggleCamStream) {
//            s_Processor = new CameraStreamProcessor();
//            visionPortal = new VisionPortal.Builder()
//                    .addProcessor(aprilTag)
//                    .addProcessor(s_Processor)
//                    .setCameraResolution(new Size(1280, 800))
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                    .setCamera(hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam))
//                    .build();
//            FtcDashboard.getInstance().startCameraStream(s_Processor, 120);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    hardwaremap.get(WebcamName.class, Constants.VisionConstants.webcam), aprilTag);
//        }
        limelight = hardwaremap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

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
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                //  Check to see if we want to track towards this tag.
                refreshDesiredID();
                if (checkDesiredTagID(fr.getFiducialId())) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = fr;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", fr.getFiducialId());
                }
            }
        return targetFound;
    }

    public void getAprilTagData(MultipleTelemetry m_telemetry) {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                refreshDesiredID();
                if (checkDesiredTagID(fr.getFiducialId())) {
                    desiredTag = fr;
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                            fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }

            }

    public void setVisionState(VisionStates.VisionState state) {
        visionStates.setState(state);
    }

    public double getTargetYaw() {
        return desiredTag.getTargetXDegrees();
    }

    public double getTargetPitch() {
        return desiredTag.getTargetYDegrees();
    }

//    public double getTargetBearing() {
//        return desiredTag.ftcPose.bearing;
//    }


    // FORMULA: d = (h2-h1) / tan(a1+a2)"
    // Distance = (Tag Height - Camera Height) / tan(Camera Angle of Elevation (From Ground) + Tag Angle of Elevation (From Camera))
    // Height (units in inches)
    public double getTargetRange() {
        return 0;
    }

//    public void setTargetY(double y) {
//        targetYaw = y;
//    }
//    public double getTargetY() {
//        return desiredTag.ftcPose.y;
//    }
//
//    public void setTargetX(double x) {
//        targetX = x;
//    }
//    public double getTargetX() {
//        return desiredTag.ftcPose.x;
//    }
//
//    public void setRobotRange(double range) {
//        robotRange = range;
//    }
//    public static double getRobotRange() {
//        return robotRange;
//    }
    public void periodic() {
        refreshDesiredID();

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

            getAprilTagData(telemetry);
    }
}
