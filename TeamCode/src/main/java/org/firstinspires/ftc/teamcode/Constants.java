package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Servo;
public class Constants {

    public static final class toggles{
        public static final boolean compMode = false;
        public static final boolean toggleCamStream = true;
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

        public static final double controlHubOffset = 90;

        public static final double maxDrive = 0.5;
        public static final double maxStrafe = 0.5;
        public static final double maxTurn = 0.5;

        @Config
        public static final class drivingPID {
            public static double driveP = 0.06;
            public static double driveI = 0.01;
            public static double driveD = 0.001;
            public static double tolY = 0.5;
            public static double strafeP = 0.08;
            public static double strafeI = 0.02;
            public static double strafeD = 0;
            public static double tolX = 0.5;
            public static double turnP = 0.008;
            public static double turnI = 0.008;
            public static double turnD = 0;
            public static double tolH = 1;
        }
    }

    public static final class IntakeConstants {

        public static final String intakeMotor = "intakeMotor";

        public static final double maxSpeed = 1.0;

        public static boolean isFull;
    }
    public static final class shooterConstants {

        public static final String shooterMotor = "shooterMotor";
        public static final double goalHeight = 0.0;
        public static final double ballTolerance = 2.5;
        public static final double shooterHeight = 0.0;
        public static final double shooterAngle = 50.0; //potentially 60.0
        public static final double maxSpeed = 1.0;
        @Config
        public static final class shooterPID {
            public static final double shooterkP = 0.0;

            public static final double shooterkI = 0.0;

            public static final double shooterkD = 0.0;
        }
    }

    public static final class VisionConstants {
        public static final String webcam = "Webcam 1";
    }

    public static final class OtosConstants {
        public static final int offsetX = 0;
        public static final int offsetY = 0;
        public static final int offsetHeading = 180;
    }

    public static final class AutoConstants {

        @Config
        public static final class AutoPoints{
            public static SparkFunOTOS.Pose2D autoOne = new SparkFunOTOS.Pose2D(0, 30, 0);
            public static SparkFunOTOS.Pose2D autoTwo = new SparkFunOTOS.Pose2D(0, 30, 180);
            public static SparkFunOTOS.Pose2D autoThree = new SparkFunOTOS.Pose2D(72, 0, 90);
            public static SparkFunOTOS.Pose2D autoFour = new SparkFunOTOS.Pose2D(0, 0, 0);
        }
    }
}
