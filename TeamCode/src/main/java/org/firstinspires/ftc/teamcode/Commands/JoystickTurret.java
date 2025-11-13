package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Turret;

import java.util.function.DoubleSupplier;

public class JoystickTurret extends CommandBase {

    Turret s_turret;
    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    public JoystickTurret(Turret s_turret, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.s_turret = s_turret;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        addRequirements(s_turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xSupplier.getAsDouble() > 0.05 || ySupplier.getAsDouble() > 0.05) {
            s_turret.setSetpoint(Math.toDegrees(Math.atan2(ySupplier.getAsDouble(), xSupplier.getAsDouble())));
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
