package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.AprilVision;

import java.util.function.BooleanSupplier;


public class ShooterCommand extends CommandBase {
    Shooter s_shooter;
    AprilVision s_aprilVision;
    BooleanSupplier shootButton;
    double desiredRPM;
    double tagDistanceMetres;
    public ShooterCommand(Shooter s_shooter, AprilVision s_AprilVision, BooleanSupplier shootButton) {
        this.s_shooter = s_shooter;
        this.s_aprilVision = s_AprilVision;
        this.shootButton = shootButton;

        addRequirements(s_shooter, s_AprilVision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(shootButton.getAsBoolean()) {
            if (s_aprilVision.foundTarget()) {
                tagDistanceMetres = s_aprilVision.getTargetRange() * 0.0254;
                desiredRPM = (111.90893 * Math.pow(tagDistanceMetres, 2)) - (85.47869 * tagDistanceMetres) + 2153.35668;
                s_shooter.setSetpoint(desiredRPM);
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}

