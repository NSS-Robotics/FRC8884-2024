package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.function.BooleanSupplier;

public class FourPiece extends BaseAuto {

    public FourPiece(
            String pathName,
            int stopPoints,
            Feeder m_feeder,
            Intake m_intake,
            Limelight l_Limelight_april,
            Pivot m_pivot,
            Shooter m_shooter,
            Swerve s_swerve,
            Candle l_candle,
            BooleanSupplier fieldmirror) {
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
                fieldmirror);
    }

    public Command shoot(boolean intake) {
        return new SequentialCommandGroup(
                intake
                        ? new NoteIntake(m_intake, m_feeder, l_candle)
                        : new InstantCommand(),
                new AimLimelight(s_swerve, l_limelight_april),
                new ParallelDeadlineGroup(
                        new SpeakerShoot(m_shooter, m_pivot, l_candle),
                        new NoteIntake(m_intake, m_feeder, l_candle)));
    }

    @Override
    public Command getCommands(Command[] swerveCommands) {
        return Commands.sequence(
                new InstantCommand(s_swerve::zeroGyro),
                shoot(false),
                swerveCommands[0],
                shoot(true),
                swerveCommands[1],
                shoot(true),
                swerveCommands[2],
                shoot(true),
                s_swerve.run(() -> s_swerve.drive(new Translation2d(0, 0), 0, false, false)));
    }
}
