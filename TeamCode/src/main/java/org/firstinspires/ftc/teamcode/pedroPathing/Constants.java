package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.97903)
            .lateralZeroPowerAcceleration(-91.3029)
            .forwardZeroPowerAcceleration(-32.9268)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.0, 0.0, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.5, 0.05, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0.001, 0.001, 0.1, 0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight1")
            .rightRearMotorName("backRight3")
            .leftRearMotorName("backLeft2")
            .leftFrontMotorName("frontLeft0")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(79.847229)
            .yVelocity(38.3027);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(6.003292E-4)
            .strafeTicksToInches(6.102184E-4)
            .turnTicksToInches(6.445494E-4)
            .leftPodY(1.0)
            .rightPodY(0.2)
            .strafePodX(2.0)
            .leftEncoder_HardwareMapName("frontLeft0")
            .rightEncoder_HardwareMapName("backRight3")
            .strafeEncoder_HardwareMapName("frontRight1")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
