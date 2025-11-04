package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.OTOS;

public class AutoCommandLine extends SequentialCommandGroup {
    Drivetrain a_drivetrain;
    OTOS a_otos;
    MultipleTelemetry m_telemetry;

    public AutoCommandLine(Drivetrain a_drivetrain, OTOS a_otos, MultipleTelemetry m_telemetry) {
        this.a_drivetrain = a_drivetrain;
        this.a_otos = a_otos;
        this.m_telemetry = m_telemetry;

        addCommands(
                new LinearAutoCommand(a_drivetrain, a_otos, m_telemetry,
                        Constants.AutoConstants.AutoPoints.autoOne),
                new WaitCommand(1000),
                new LinearAutoCommand(a_drivetrain, a_otos, m_telemetry,
                        Constants.AutoConstants.AutoPoints.autoTwo),
                new WaitCommand(1000),
//                new LinearAutoCommand(a_drivetrain, a_otos, m_telemetry,
//                        Constants.AutoConstants.AutoPoints.autoThree),
//                new WaitCommand(1000),
//                new LinearAutoCommand(a_drivetrain, a_otos, m_telemetry,
//                        Constants.AutoConstants.AutoPoints.autoFour),
//                new WaitCommand(1000),
                new KhabyLameMechanism(a_drivetrain, a_otos)

        );

        addRequirements(a_drivetrain);

    }
}
