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
    public static double yaw;

    public static Pose3D robotPose;

    public int goodTagID;


    public AprilVision(HardwareMap hardwaremap, MultipleTelemetry telemetry, VisionStates visionState, int goodTagID) {

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
                    robotPose = fr.getRobotPoseFieldSpace();

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

    public Pose3D getRobotPose() {
        return robotPose;
    }

    public void periodic() {
        refreshDesiredID();
        fifo.add(getTargetRange());

        LLStatus status = limelight.getStatus();
        telemetry.addData("targetFound", foundTarget());
            getAprilTagData(telemetry);



        telemetry.addData("TARGET RANGE AVG:", getRangeAvg());
        telemetry.addData("ROBOT POSE: ", robotPose);
    }
}
