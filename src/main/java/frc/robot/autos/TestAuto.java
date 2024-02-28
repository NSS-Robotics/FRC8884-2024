package frc.robot.autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TestAuto extends BaseAuto {
    public TestAuto(
            String pathName,
            int stopPoints,
            Feeder m_feeder,
            Intake m_intake,
            Limelight l_Limelight,
            Pivot m_pivot,
            Shooter m_shooter,
            Swerve s_swerve,
            BooleanSupplier fieldmirror) {
        super(pathName, stopPoints, m_feeder, m_intake, l_Limelight, m_pivot, m_shooter, s_swerve, fieldmirror);
    }

    @Override
    public Command getCommands(Command[] swerveCommands) {
        return Commands.sequence(
                new InstantCommand(s_Swerve::zeroHeading),
                swerveCommands[0],
                swerveCommands[1],
                s_Swerve.run(() -> s_Swerve.drive(
                        new Translation2d(0, 0),
                        0,
                        false,
                        false)));
    }
}