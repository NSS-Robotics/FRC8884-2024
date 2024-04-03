package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SpeakerShootForAuto extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Feeder feeder;
    private final Candle candle;

    public SpeakerShootForAuto(Shooter _shooter, Pivot _pivot, Feeder _feeder, Candle candle) {
        shooter = _shooter;
        pivot = _pivot;
        feeder = _feeder;
        this.candle = candle;
        addRequirements(shooter, pivot);
    }

    @Override
    public void execute() {
        feeder.setShouldShoot(true);
        double rotations = Math.min(1,(Math.max(0, pivot.getRotations())));
        // pivot.setPivot(Constants.PivotConstants.PivotAgainstRotations);
        pivot.setPivot(rotations);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setHasBeenDetected(false);
        feeder.setShouldRev(false);
        feeder.setLemmeShootBro(false);
        feeder.setShouldShoot(false);
        candle.ledsOff();
        shooter.stop();
        feeder.stop();
    }
}
