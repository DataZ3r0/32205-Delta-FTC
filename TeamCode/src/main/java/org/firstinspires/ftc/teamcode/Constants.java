package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

        public static final double driveTolerance = 4;

        public static final double rotationTolerance = 2;

//        public static enum rotatingDirections{
//            CLOCKWISE,
//            COUNTER_CLOCKWISE,
//            NONE
//        }
        @Config
        public static final class drivePID {
            public static double kPdrive = 0.02;
            public static double kPstrafe = 0.015;
            public static double kPturn = 0.05;
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
        public static final double shooterRPMTolerance = 100;
        public static final double goalHeight = 0.0;
        public static final double ballTolerance = 2.5;
        public static final double shooterHeight = 0.0;
        public static final double shooterAngle = 50.0; //potentially 60.0


        @Config
        public static final class shooterConfigs {
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
            public static SparkFunOTOS.Pose2D startpos = new SparkFunOTOS.Pose2D(0,0, 0);
            public static SparkFunOTOS.Pose2D autoOne = new SparkFunOTOS.Pose2D(-30, -30, 45);
            public static SparkFunOTOS.Pose2D autoTwo = new SparkFunOTOS.Pose2D(0, 27, 0);
            public static SparkFunOTOS.Pose2D autoThree = new SparkFunOTOS.Pose2D(0, -27, 45);
            public static SparkFunOTOS.Pose2D autoFour = new SparkFunOTOS.Pose2D(0, 0, 0);
        }
    }
}
