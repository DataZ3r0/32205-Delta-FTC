package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
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
