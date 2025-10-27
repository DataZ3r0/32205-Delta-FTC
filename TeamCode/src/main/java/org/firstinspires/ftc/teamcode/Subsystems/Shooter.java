package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Shooter extends SubsystemBase {

    private final DcMotor shooterMotor;
    public Shooter(HardwareMap hardwaremap) {
        shooterMotor = hardwaremap.get(DcMotor.class, Constants.shooterConstants.shooterMotor);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runShooter(double speed) {
        shooterMotor.setPower(speed * Constants.shooterConstants.maxSpeed);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }
    public void periodic() {

    }
}
