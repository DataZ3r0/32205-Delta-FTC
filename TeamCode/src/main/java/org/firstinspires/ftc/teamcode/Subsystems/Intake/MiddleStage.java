package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class MiddleStage extends SubsystemBase {

    private final DcMotorEx motor;
    private DcMotorSimple.Direction direction;

    public MiddleStage(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.middleStageMotor);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPower() {
        return motor.getPower();
    }

    public void runIntake(double power) {
        motor.setPower(power);
    }

    public void runOuttake(double power) {
        motor.setPower(-power);
    }
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic() {}
}
