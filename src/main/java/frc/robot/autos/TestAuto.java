package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.function.BooleanSupplier;

public class TestAuto extends BaseAuto {

    public TestAuto(
        String pathName,
        int stopPoints,
        Feeder m_feeder,
        Intake m_intake,
        Limelight l_Limelight_april,
        Pivot m_pivot,
        Shooter m_shooter,
        Swerve s_swerve,
        Candle l_candle,
        BooleanSupplier fieldmirror
    ) {
        super(
            pathName,
            stopPoints,
            m_feeder,
            m_intake,
            l_Limelight_april,
            m_pivot,
            m_shooter,
            s_swerve,
            l_candle,
            fieldmirror
        );
    }

    @Override
    public Command getCommands(Command[] swerveCommands) {
        return Commands.sequence(
            new InstantCommand(s_swerve::zeroGyro),
            swerveCommands[0],
            s_swerve.run(() ->
                s_swerve.drive(new Translation2d(0, 0), 0, false, false)
            )
        );
    }
}
