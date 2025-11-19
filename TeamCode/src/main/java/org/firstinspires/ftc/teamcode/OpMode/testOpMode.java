package org.firstinspires.ftc.teamcode.OpMode;
/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.Commands.JoystickTurret;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.MiddleStage;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.GlobalPoseEstimation;
import org.firstinspires.ftc.teamcode.VisionStates;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
 * with the FTC VisionPortal.   It captures a camera frame (image) and stores it on the Robot Controller
 * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
 * Full calibration instructions are here:
 *
 *  https://ftc-docs.firstinspires.org/camera-calibration
 *
 * In Android Studio, copy this class into your "teamcode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * In OnBot Java, use "Add File" to add this OpMode from the list of Samples.
 */

@TeleOp(name = "testOpModeDelta", group = "Utility")

public class testOpMode extends LinearOpMode
{
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
//    final boolean USING_WEBCAM = true;
//    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
//    final int RESOLUTION_WIDTH = 1280;
//    final int RESOLUTION_HEIGHT = 800;
    MultipleTelemetry m_telemetry;

    // Internal state
//    boolean lastX;
//    int frameCount;
//    long capReqTime;
//    private AprilTagProcessor aprilTag;
//    private CameraStreamProcessor s_Processor;
    GamepadEx opGamepad;
//    AprilVision s_aprilVision;
//    VisionStates visionState;
//    Intake s_intake;
//    MiddleStage s_middleStage;
    Turret s_turret;
//    GlobalPoseEstimation poseEstimation;
//    OTOS s_otos;
//
//
//
//    Shooter s_shooter;

    @Override
    public void runOpMode()
    {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        opGamepad = new GamepadEx(gamepad1);
//        visionState = new VisionStates();
//        s_shooter = new Shooter(hardwareMap, m_telemetry);
//        s_intake = new Intake(hardwareMap);
//        s_middleStage = new MiddleStage(hardwareMap);
//        s_turret = new Turret(hardwareMap, s_dr m_telemetry);
//        s_otos = new OTOS(hardwareMap, m_telemetry);
//        poseEstimation = new GlobalPoseEstimation(s_otos, s_aprilVision, s_turret);

//        visionState.setState(VisionStates.VisionState.SHOOT);
//        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState);
        CommandScheduler.getInstance().run();

        waitForStart();

        while(opModeIsActive()) {
            if(!Constants.toggles.manTurret) {
//                new RunCommand(() -> {
//                double deltaX = Constants.toggles.blueTeam ? Constants.FieldConstants.blueGoal.x - poseEstimation.getPose().x : Constants.FieldConstants.redGoal.x - poseEstimation.getPose().x;
//                double deltaY = Constants.toggles.blueTeam ? Constants.FieldConstants.blueGoal.y - poseEstimation.getPose().y : Constants.FieldConstants.redGoal.y - poseEstimation.getPose().y;
//                s_turret.setSetpoint(Math.toDegrees(Math.tan(deltaY/deltaX)));
//            }, s_turret, poseEstimation);
            } else {
//                if(Math.abs(opGamepad.getRightX()) >= 0.5 || Math.abs(opGamepad.getRightY()) >= 0.5 && false;) {
//                    s_turret.manuelTurret(
//                            opGamepad.getRightX(), opGamepad.getRightY()
//                    );
//                }
//                else {
//                    s_turret.manuelTurret(0,0);
//                }
            }

//            s_turret.setDefaultCommand(
//                    new JoystickTurret(s_turret, () -> opGamepad.getRightX(), ()-> opGamepad.getRightY())
//            );

//            s_aprilVision.periodic();
            m_telemetry.addData("right stick x", opGamepad.getRightX());
            m_telemetry.addData("right stick y", opGamepad.getRightY());
            m_telemetry.addData("right stick angle", s_turret.getJoystickAngle(opGamepad.getRightX(), opGamepad.getRightY()));

            s_turret.periodic();
            m_telemetry.update();
        }
    }
}

