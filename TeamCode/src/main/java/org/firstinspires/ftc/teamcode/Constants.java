package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
        public static final class drivePID {
            public static double kPdrive = 0.02;
            public static double kPstrafe = 0.015;
            public static double kPturn = 0.05;
        }
    }

    public static final class IntakeConstants {

        public static final String intakeMotor = "intakeMotor";

        public static final double maxSpeed = 1.0;
    }
    public static final class shooterConstants {

        public static final String intakeMotor = "shooterMotor";


        public static final double maxSpeed = 1.0;
    }

    public static final class VisionConstants {
        public static final String webcam = "Webcam 1";
    }

    public static final class OtosConstants {
        public static final int offsetX = 0;
        public static final int offsetY = 1;
        public static final int offsetHeading = 180;
    }

    public static final class AutoConstants {

        @Config
        public static final class AutoStart{
            public static double autoTargetX = 10;
            public static double autoTargetY = 10;
            public static double autoTargetH = 90;
        }
    }
}
