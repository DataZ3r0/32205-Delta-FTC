package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    private final DcMotor intakeMotor;
    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, Constants.IntakeConstants.intakeMotor);
    }
}
