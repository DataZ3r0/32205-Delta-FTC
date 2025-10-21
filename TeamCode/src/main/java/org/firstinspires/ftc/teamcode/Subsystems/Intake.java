package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private DcMotorSimple.Direction direction;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intakeMotor);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void outtake() {
        direction = DcMotorSimple.Direction.REVERSE;
    }

    public void intake() {
        direction = DcMotorSimple.Direction.FORWARD;
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public void run() {
        intakeMotor.setPower(1);
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


    public void periodic() {

    }
}
