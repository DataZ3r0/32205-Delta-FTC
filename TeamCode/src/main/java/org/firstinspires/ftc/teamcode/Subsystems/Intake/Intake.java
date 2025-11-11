package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;

public class Intake extends SubsystemBase {
    private final DcMotorEx intakeMotor;
    private DcMotorSimple.Direction direction;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.intakeMotor);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void runOuttake(double power) {
        intakeMotor.setPower(-power);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}
