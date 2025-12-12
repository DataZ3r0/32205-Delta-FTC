package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.MiddleStage;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;

public class AutoIntakeCommand {

    Intake s_intake;
    MiddleStage s_middle;
    Shooter s_shooter;

    boolean isCalled;

    public AutoIntakeCommand(Intake s_intake, MiddleStage s_middle, Shooter s_shooter) {
        this.s_intake = s_intake;
        this.s_middle = s_middle;
        this.s_shooter = s_shooter;
    }

    public void enable() {
        isCalled = true;
    }

    public void disable() {
        isCalled = false;
    }

    public void periodic() {
        if (isCalled) {
            s_intake.runIntake(1);
            s_middle.runIntake(1);
            s_shooter.runLoader();
        } else {
            s_intake.stop();
            s_middle.stop();
            s_shooter.stopLoader();
        }
    }
}
