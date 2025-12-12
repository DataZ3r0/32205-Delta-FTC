/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
public class OTOS extends SubsystemBase {
    // Create an instance of the sensor
    SparkFunOTOS otos;
    double headingOffset;

    SparkFunOTOS.Pose2D pos;

    MultipleTelemetry telemetry;

    public OTOS(HardwareMap hardwareMap, MultipleTelemetry telemetry) {

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.setLinearScalar(1.22772277228);
        otos.setAngularScalar(0.99833610648);

        this.telemetry = telemetry;
        configureOtos(telemetry);
    }

    @SuppressLint("DefaultLocale")
    private void configureOtos(Telemetry telemetry) {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(
                Constants.OtosConstants.offsetX,
                Constants.OtosConstants.offsetY,
                Constants.OtosConstants.offsetHeading);
        otos.setOffset(offset);

        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public double getX() {
        return pos.x * 2.22772277228;
    }
    public double getY() { return pos.y * 2.22772277228; }
    public double getH() {
        return -pos.h * 0.99833610648;
    }

    public void setPose(SparkFunOTOS.Pose2D newPose) {
        otos.setPosition(newPose);
    }

    public void resetOTOS() { otos.resetTracking(); }

    public SparkFunOTOS.Pose2D getPose() {
        return otos.getPosition();
    }

    public void periodic() {
        pos = getPose();
        telemetry.addData("OTOS X", getX());
        telemetry.addData("OTOS Y", getY());
        telemetry.addData("OTOS HEADING", getH());
    }
//    public void autoPeriodic(MultipleTelemetry m_telemetry, SparkFunOTOS.Pose2D targetPose) {
//        m_telemetry.addData("OTOS X", targetPose.x);
//        m_telemetry.addData("OTOS Y", targetPose.y);
//        m_telemetry.addData("OTOS HEADING", targetPose.h);
//    }
}
