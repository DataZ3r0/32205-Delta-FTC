package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
public class Constants {
    public static final boolean compMode = false;
    public static final class DrivetrainConstants {
        public static final String frontLeftMotor = "frontLeft0";
        public static final String frontRightMotor = "frontRight1";
        public static final String backLeftMotor = "backLeft2";
        public static final String backRightMotor = "backRight3";

        public static final double strafingBalancer = 1.1;

        public static final double controlHubOffset = 90;

        public static final double kPdrive = 0.02;
        public static final double kPstrafe = 0.015;
        public static final double kPturn = 0.05;

        public static final double maxDrive = 0.5;
        public static final double maxStrafe = 0.5;
        public static final double maxTurn = 0.5;

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
}
