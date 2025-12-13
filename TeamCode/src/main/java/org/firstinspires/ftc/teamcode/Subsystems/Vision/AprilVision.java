package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.LimitedQueue;
import org.firstinspires.ftc.teamcode.VisionStates;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;
import java.util.Queue;

public class AprilVision extends SubsystemBase {

    private final MultipleTelemetry telemetry;
    private int[] desiredTagID;
//    private final AprilTagProcessor aprilTag;
//    private final VisionPortal visionPortal;
//    private final CameraStreamProcessor s_Processor;
    private final VisionStates visionStates;
    public AprilTagPoseFtc ftcPose;
    public LLResultTypes.FiducialResult desiredTag;

    public LimitedQueue<Double> fifo;

    private Limelight3A limelight;

    public boolean targetFound;

    public static double targetRange;
    public static double targetYaw;
    public static double targetBearing;
    public static double tY;
    public static double tX;

    public int goodTagID;


//    public LLResult result;
    //Fx/Fy = 946.233
    //Cx = 667.521
    //Cy = 464.348
//    Radial distortion (Brown's Model)
//            K1: 0.0607443 K2: 0.0624121 K3: -0.303675
//            P1: 0.0142314 P2: 0.00530697
//            Skew: 0
    //Mean Square Reprojection Error: 0.433367 pixels

    public AprilVision(HardwareMap hardwaremap, MultipleTelemetry telemetry, VisionStates visionState, int goodTagID) {
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
        limelight = hardwaremap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        if (goodTagID == 20) {
            limelight.pipelineSwitch(0);
        } else if (goodTagID == 24){
            limelight.pipelineSwitch(1);
        }

        this.goodTagID = goodTagID;


        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        this.telemetry = telemetry;
        this.visionStates = visionState;
        this.goodTagID = goodTagID;
        fifo = new LimitedQueue<>(Constants.VisionConstants.listLength);
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
        if (tagID == goodTagID) {
            return true;
        } else {
            return false;
        }
    }

    public void getAprilTagData(MultipleTelemetry m_telemetry) {
        targetFound = false;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()){


            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                m_telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                refreshDesiredID();
                m_telemetry.addData("CORRECT TAG ID?", checkDesiredTagID(fr.getFiducialId()));
                if (checkDesiredTagID(goodTagID)) {
                    targetFound = true;
                    desiredTag = fr;
                    tX = fr.getTargetXDegrees();
                    tY = fr.getTargetYDegrees();
                } else {
                    m_telemetry.addData("Skipping", "Tag ID %d is not desired", fr.getFiducialId());
                }
            }
        }


    }

    public boolean foundTarget() {
        return targetFound;
    }

    public void setVisionState(VisionStates.VisionState state) {
        visionStates.setState(state);
    }

    public double getTx() {
        return tX;
    }

    public double getTy() {
        return tY;
    }

//    public double getTargetBearing() {
//        return desiredTag.ftcPose.bearing;
//    }


    // FORMULA: d = (h2-h1) / tan(a1+a2)"
    // Distance = (Tag Height - Camera Height) / tan(Camera Angle of Elevation (From Ground) + Tag Angle of Elevation (From Camera))
    // Height (units in inches)
    // shooter height 14.491 tag height 29.5
    public double getTargetRange() {
        double angleToGoalRadians = Math.toRadians(20 + getTy());
        return (29.5 - 14.241)/(Math.tan(angleToGoalRadians));
    }

    public double getRangeAvg() {
        double totalvalue = 0;
        for (int i = 0; i < fifo.size(); i++) {
            totalvalue += fifo.get(i);
        }
        return totalvalue/fifo.size();
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
        fifo.add(getTargetRange());

        LLStatus status = limelight.getStatus();
//        telemetry.addData("Name", "%s",
//                status.getName());
//        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("targetFound", foundTarget());
            getAprilTagData(telemetry);



        telemetry.addData("TARGET RANGE AVG:", getRangeAvg());
//        telemetry.addData("reult", limelight.getLatestResult());
//            telemetry.addData("is result vaid", result.isValid());
    }
}
