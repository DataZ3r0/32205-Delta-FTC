package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;

public class SmartIntake extends CommandBase {
    private final distanceSensor s_ds;
    private MultipleTelemetry telemetry;
    private final Intake s_intake;
    private final Shooter s_shooter;
    private double pieceCheckThreshold;
    private int ballCount;
    public SmartIntake(Intake s_intake, distanceSensor s_ds, Shooter s_shooter) {
        this.s_intake = s_intake;
        this.s_ds = s_ds;
        this.s_shooter = s_shooter;
        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        pieceCheckThreshold = 10; //inches change later
        ballCount = 0;
    }

    @Override
    public void execute() {
        if(s_ds.getDistance() < pieceCheckThreshold) {
            ballCount++;
        }
        if(s_shooter.wasBallShot()){
            ballCount--;
        }
        Constants.IntakeConstants.isFull = ballCount >= 3;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
