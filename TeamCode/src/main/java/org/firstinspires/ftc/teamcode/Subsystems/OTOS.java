/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode.Subsystems;

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

    SparkFunOTOS.Pose2D pos;

    public OTOS(HardwareMap hardwareMap, Telemetry telemetry) {

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos(telemetry);
    }

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

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

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
        return pos.x;
    }
    public double getY() {
        return pos.y;
    }
    public double getH() {
        return pos.h;
    }

    public SparkFunOTOS.Pose2D getPose() {
        return otos.getPosition();
    }

    public void periodic(MultipleTelemetry m_telemetry) {
        pos = otos.getPosition();
        m_telemetry.addData("OTOS X", pos.x);
        m_telemetry.addData("OTOS Y", pos.y);
        m_telemetry.addData("OTOS HEADING", pos.h);
    }
}
