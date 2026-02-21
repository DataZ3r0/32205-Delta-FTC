package org.firstinspires.ftc.teamcode.OpMode; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.TurretCalculator;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.MiddleStage;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.NewTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.GlobalPoseEstimation;
import org.firstinspires.ftc.teamcode.VisionStates;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class newauto extends OpMode {
    MultipleTelemetry m_telemetry;
    Drivetrain s_drivetrain;
    AprilVision s_aprilVision;
    Intake s_intake;
    MiddleStage s_middleStage;
    Shooter s_shooter;
    NewTurret s_turret;
    Odometry s_odometry;

    VisionStates visionState;
    TurretCalculator turretCalculator;
    AutoIntakeCommand intakeCommand;

    private boolean allowShoot;

    private boolean allowIntake;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.922, 112.390),

                                new Pose(62.283, 83.824)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(62.283, 83.824),

                                new Pose(23.478, 84.688)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.478, 84.688),

                                new Pose(17.327, 75.161)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.327, 75.161),

                                new Pose(62.663, 83.980)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.663, 83.980),
                                new Pose(55.441, 58.710),
                                new Pose(23.673, 60.327)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.673, 60.327),

                                new Pose(16.859, 70.946)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.859, 70.946),

                                new Pose(62.151, 82.327)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.151, 82.327),
                                new Pose(60.132, 32.480),
                                new Pose(23.883, 35.590)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.883, 35.590),

                                new Pose(63.102, 84.302)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.102, 84.302),

                                new Pose(64.156, 110.517)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(1);
                }
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path2,true);
                    allowIntake = true;
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path3,true);
                    allowIntake = false;
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path4,true);
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path5,true);
                    allowIntake = true;
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Path6,true);
                    allowIntake = false;
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path7, true);
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path8, true);
                    setPathState(8);
                    allowIntake = true;
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path9, true);
                    setPathState(9);
                    allowIntake = false;
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(Path10, true);
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        setPathState(10);
                    }
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        s_drivetrain.periodic();
        s_aprilVision.periodic();
        s_shooter.periodic();
        s_turret.periodic();

        intakeCommand.periodic();

        s_turret.setSetpoint(-s_drivetrain.getHeading());
        s_shooter.setDesiredVelocity(turretCalculator.getDistanceFromTarget(
                new SparkFunOTOS.Pose2D(s_odometry.getX(), s_odometry.getY(), s_odometry.getH()),
                new SparkFunOTOS.Pose2D(-72, 72, 0)));

        if (s_aprilVision.targetFound) {
            allowShoot = true;
        } else {
            allowShoot = false;
        }

        s_shooter.stopperAutoPeriodic(allowShoot);

        if (allowShoot || allowIntake) {
            intakeCommand.enable();
        } else {
            intakeCommand.disable();
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        allowIntake = false;
        allowShoot = false;

        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionState = new VisionStates();

        s_drivetrain = new Drivetrain(hardwareMap, m_telemetry);
        s_aprilVision = new AprilVision(hardwareMap, m_telemetry, visionState, 20);
        s_intake = new Intake(hardwareMap);
        s_middleStage = new MiddleStage(hardwareMap);
        s_shooter = new Shooter(hardwareMap, m_telemetry, false);
        s_turret = new NewTurret(hardwareMap, s_drivetrain, m_telemetry);

        s_odometry = new Odometry(hardwareMap, Constants.localizerConstants, m_telemetry);

        intakeCommand = new AutoIntakeCommand(s_intake, s_middleStage, s_shooter); //6.4902843E-4
        turretCalculator = new TurretCalculator(s_odometry, s_aprilVision, s_turret);


        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(23.736, 128.906));

        s_turret.stopTurret();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}




}

