package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SpeakerShootForAuto extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Candle candle;
    private final Feeder feeder;

    public SpeakerShootForAuto(Shooter _shooter, Pivot _pivot, Candle candle, Feeder feeder) {
        this.shooter = _shooter;
        this.pivot = _pivot;
        this.candle = candle;
        this.feeder = feeder;
        addRequirements(shooter, pivot, candle, feeder);
    }

    @Override
    public void execute() {

    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        shooter.stop();
        feeder.stop();
        candle.setLEDs(170, 247, 250);
    }
}
