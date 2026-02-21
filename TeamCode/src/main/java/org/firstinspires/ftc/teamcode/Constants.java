package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


// RED GOAL X: 57.85399 Y:60.67376
// BLUE GOAL X: -57.85399 Y:60.67376
public class Constants {
    @Config
    public static final class toggles{
        public static boolean compMode = false;
        public static boolean toggleCamStream = true;
        public static boolean blueTeam = true;
        public static boolean manTurret = true;
    }

    public static final class FieldConstants {
        public static final SparkFunOTOS.Pose2D blueGoal = new SparkFunOTOS.Pose2D(0,0,0);
        public static final SparkFunOTOS.Pose2D redGoal = new SparkFunOTOS.Pose2D(0,0,0);
        public static final SparkFunOTOS.Pose2D motif = new SparkFunOTOS.Pose2D(0,0,0);
    }

    public static final class DrivetrainConstants {
        public static final String frontLeftMotor = "frontLeft0";
        public static final String frontRightMotor = "frontRight1";
        public static final String backLeftMotor = "backLeft2";
        public static final String backRightMotor = "backRight3";

        public static final double CountsPerMotorRev = 435.0;   // eg: GoBILDA 312 RPM Yellow Jacket
        public static final double DriveGearReduction = 1.0;     // No External Gearing.
        public static final double WheelDiameterInches = 4.0;     // For figuring circumference
        public static final double CountsPerInch = (CountsPerMotorRev * DriveGearReduction /
                (WheelDiameterInches * 3.1415));

        public static final double strafingBalancer = 1.0;

        public static final double controlHubOffset = 180;

        public static final double maxDrive = 0.5;
        public static final double maxStrafe = 0.5;
        public static final double maxTurn = 0.5;

        public static final double driveTolerance = 5;

        public static final double rotationTolerance = 3;

//        public static enum rotatingDirections{
//            CLOCKWISE,
//            COUNTER_CLOCKWISE,
//            NONE
//        }
        @Config
        public static final class drivePID {
            public static double drivekP = 0.15;
            public static double drivekI = 0.08;
            public static double drivekD = 0.005;
            public static double turnkP = 0.02;
            public static double turnkI = 0.01;
            public static double turnkD = 0.0001;
        }
    }

    public static final class IntakeConstants {

        public static final String intakeMotor = "intakeMotor0";
        public static final String middleStageMotor = "midStageMotor2";

        public static final double maxSpeed = 1.0;

        public static boolean isFull;
    }
    public static final class shooterConstants {

        public static final String shooterMotor = "shooterMotor3";
        public static final String loadingServo = "loadingServo";
        public static boolean loadingServoRev = false;
        public static double loadingServoSpeed = 1; //degrees
        public static final double ticksPerRev = 28;
        public static final double shooterRPMTolerance = 50;
        public static final double goalHeight = 0.0;
        public static final double ballTolerance = 2.5;
        public static final double shooterHeight = 0.0;
        public static final double shooterAngle = 50.0; //potentially 60.0


        @Config
        public static final class shooterConfigs {
            public static double kShoot = 1.00;
            public static double testRPM = 2400;
            public static double maxSpeed = 1;
            public static double shooterkP = 0.001;
            public static double shooterkI = 0.06;
            public static double shooterkD = 0.0;
            public static double shooterkS = 0.003;
            public static double shooterkV = 0.000195;

        }
    }

    public static final class turretConstants {
        public static final String turretMotor = "turretMotor1";
        @Config
        public static final class turretConfigs {
            public static double maxSpeed = 1.0;
            public static double turretkP = 0.03;
            public static double turretkI = 0;
            public static double turretkD = 0.00000001;

            public static double turretSetPoint = 0;
        }
    }

    public static final class VisionConstants {
        public static final String webcam = "Limelight";
        public static final int listLength = 20;
    }

    public static final class OtosConstants {
        public static final int offsetX = 0;
        public static final int offsetY = 0;
        public static final int offsetHeading = 0;
    }

    public static final class OdometryConstants {
        public static final double xTicksPerInch = 1.0;
        // sideToSide
        public static final double yTicksPerInch = 1.0;
        // forwardToBack
        public static final double rotTicksPerInch = 1.0;
    }

    public static class RedPaths {
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

        public RedPaths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.078, 112.156),

                                    new Pose(82.654, 83.590)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.654, 83.590),

                                    new Pose(118.541, 83.751)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.541, 83.751),

                                    new Pose(127.361, 72.868)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.361, 72.868),

                                    new Pose(83.268, 83.044)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(83.268, 83.044),
                                    new Pose(82.134, 56.134),
                                    new Pose(120.610, 58.688)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.610, 58.688),

                                    new Pose(127.161, 70.239)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.161, 70.239),

                                    new Pose(82.522, 83.263)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(82.522, 83.263),
                                    new Pose(75.820, 31.778),
                                    new Pose(119.654, 36.273)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.654, 36.273),

                                    new Pose(81.834, 83.132)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.834, 83.132),

                                    new Pose(82.185, 110.751)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();
        }
    }


    public static class BluePaths {
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

        public BluePaths(Follower follower) {
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
    }

    public static class FarBluePaths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public FarBluePaths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.234, 17.971)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.234, 17.971),
                                    new Pose(13.759, 24.863),
                                    new Pose(11.000, 11.941)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.000, 11.941),

                                    new Pose(56.195, 17.761)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(110))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.195, 17.761),
                                    new Pose(14.283, 6.556),
                                    new Pose(12.644, 15.454)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.644, 15.454),

                                    new Pose(55.951, 17.771)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.951, 17.771),

                                    new Pose(31.595, 17.410)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


    public static class FarRedPaths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public FarRedPaths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.800, 9.873),

                                    new Pose(84.800, 20.078)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.800, 20.078),
                                    new Pose(127.320, 26.268),
                                    new Pose(133.224, 12.176)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.224, 12.176),

                                    new Pose(84.761, 20.605)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(70))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.761, 20.605),
                                    new Pose(125.971, 4.449),
                                    new Pose(133.698, 17.327)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.698, 17.327),

                                    new Pose(84.985, 20.580)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.985, 20.580),

                                    new Pose(117.527, 14.600)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
        }
    }


    public static final class AutoConstants {

        @Config
        public static final class AutoPoints{
            public static Pose startPose =  new Pose(0,0,0);
            public static Pose shootPose =  new Pose(0,0,0);
            public static Pose pickupEntry1 =  new Pose(0,0,0);
            public static Pose pickupEntry2 = new Pose(0,0,0);
            public static Pose pickupEntry3 = new Pose(0,0,0);
            public static Pose pickupEnd1 = new Pose(0,0,0);
            public static Pose pickupEnd2 = new Pose(0,0,0);
            public static Pose pickupEnd3 = new Pose(0,0,0);


            public static SparkFunOTOS.Pose2D testPoint1 = new SparkFunOTOS.Pose2D(30,30, 45);
            public static SparkFunOTOS.Pose2D testPoint2 = new SparkFunOTOS.Pose2D(0,0, 0);
            public static SparkFunOTOS.Pose2D startpos = new SparkFunOTOS.Pose2D(0,0, 0);
            public static SparkFunOTOS.Pose2D autoOne = new SparkFunOTOS.Pose2D(-39, -47, 35);
            public static SparkFunOTOS.Pose2D autoTwo = new SparkFunOTOS.Pose2D(0, 53, 0);
            public static SparkFunOTOS.Pose2D autoThree = new SparkFunOTOS.Pose2D(0, -53, 0);
            public static SparkFunOTOS.Pose2D autoFour = new SparkFunOTOS.Pose2D(0, 0, 45);
            public static SparkFunOTOS.Pose2D autoFive = new SparkFunOTOS.Pose2D(-32, -5, 80);
            public static SparkFunOTOS.Pose2D autoSix = new SparkFunOTOS.Pose2D(0, 63, 0);
            public static SparkFunOTOS.Pose2D autoSeven = new SparkFunOTOS.Pose2D(42, -60, -30);
            public static SparkFunOTOS.Pose2D autoEight = new SparkFunOTOS.Pose2D(0, 0, 45);
            public static SparkFunOTOS.Pose2D autoNine = new SparkFunOTOS.Pose2D(-70, 0, 90);
            public static SparkFunOTOS.Pose2D autoTen = new SparkFunOTOS.Pose2D(0, 49, 0);
            public static SparkFunOTOS.Pose2D autoEleven = new SparkFunOTOS.Pose2D(70, -49, -55);
            public static SparkFunOTOS.Pose2D autoTwelve = new SparkFunOTOS.Pose2D(0, 0, 45);

            public static SparkFunOTOS.Pose2D redautoOne = new SparkFunOTOS.Pose2D(39, -47, -35);
            public static SparkFunOTOS.Pose2D redautoTwo = new SparkFunOTOS.Pose2D(0, 53, 0);
            public static SparkFunOTOS.Pose2D redautoThree = new SparkFunOTOS.Pose2D(0, -53, 0);
            public static SparkFunOTOS.Pose2D redautoFour = new SparkFunOTOS.Pose2D(0, 0, -45);
            public static SparkFunOTOS.Pose2D redautoFive = new SparkFunOTOS.Pose2D(32, -5, -80);
            public static SparkFunOTOS.Pose2D redautoSix = new SparkFunOTOS.Pose2D(0, 63, 0);
            public static SparkFunOTOS.Pose2D redautoSeven = new SparkFunOTOS.Pose2D(-42, -60, 45);
            public static SparkFunOTOS.Pose2D redautoEight = new SparkFunOTOS.Pose2D(0, 0, -45);
            public static SparkFunOTOS.Pose2D redautoNine = new SparkFunOTOS.Pose2D(70, 0, -90);
            public static SparkFunOTOS.Pose2D redautoTen = new SparkFunOTOS.Pose2D(0, 49, 0);
            public static SparkFunOTOS.Pose2D redautoEleven = new SparkFunOTOS.Pose2D(-70, -49, 55);
            public static SparkFunOTOS.Pose2D redautoTwelve = new SparkFunOTOS.Pose2D(0, 0, -45);

            public static SparkFunOTOS.Pose2D farautoOne = new SparkFunOTOS.Pose2D(-10, 10, -15);
            public static SparkFunOTOS.Pose2D farautoTwo = new SparkFunOTOS.Pose2D(0, 28, 0);
            public static SparkFunOTOS.Pose2D farautoThree = new SparkFunOTOS.Pose2D(-56, 0, -90);
            public static SparkFunOTOS.Pose2D farautoFour = new SparkFunOTOS.Pose2D(56, -28, -45);
            public static SparkFunOTOS.Pose2D farautoFive = new SparkFunOTOS.Pose2D(0, 0, -15);
            public static SparkFunOTOS.Pose2D farautoSix = new SparkFunOTOS.Pose2D(-53, 0, -90);

            public static SparkFunOTOS.Pose2D redfarautoOne = new SparkFunOTOS.Pose2D(10, 10, 15);
            public static SparkFunOTOS.Pose2D redfarautoTwo = new SparkFunOTOS.Pose2D(0, 28, 0);
            public static SparkFunOTOS.Pose2D redfarautoThree = new SparkFunOTOS.Pose2D(56, 0, 90);
            public static SparkFunOTOS.Pose2D redfarautoFour = new SparkFunOTOS.Pose2D(-56, -28, 45);
            public static SparkFunOTOS.Pose2D redfarautoFive = new SparkFunOTOS.Pose2D(0, 0, 15);
            public static SparkFunOTOS.Pose2D redfarautoSix = new SparkFunOTOS.Pose2D(53, 0, 90);

            public static SparkFunOTOS.Pose2D gateautoOne = new SparkFunOTOS.Pose2D(0, -25, 90);
            public static SparkFunOTOS.Pose2D gateautoTwo = new SparkFunOTOS.Pose2D(0, -25, 90);


            public static SparkFunOTOS.Pose2D redgateautoOne = new SparkFunOTOS.Pose2D(0, -25, -90);
            public static SparkFunOTOS.Pose2D redgateautoTwo = new SparkFunOTOS.Pose2D(0, -25, -90);



            
        }
    }
}
