package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private DcMotorSimple.Direction direction;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intakeMotor);
        intakeMotor.setDirection(direction);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void toggleOuttakeMode(boolean reversed) {
        if(!reversed) {
            direction = DcMotorSimple.Direction.REVERSE;
        } else {
            direction = DcMotorSimple.Direction.FORWARD;
        }
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void run() {
        intakeMotor.setPower(1);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }


    public void periodic() {

    }
}
