package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AmpShoot extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Candle candle;
    private final Feeder feeder;

    public AmpShoot(Shooter shooter, Pivot pivot, Feeder feeder, Candle candle) {
        this.shooter = shooter;
        this.pivot = pivot;
        this.feeder = feeder;
        this.candle = candle;
        addRequirements(shooter, pivot);
    }

    @Override
    public void execute() {
        if (feeder.getShouldRev()) {
            feeder.setShouldRev(false);
            feeder.setShouldAmp(true);
        }
        pivot.setPivot(pivot.getAmp());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setHasBeenDetected(false);
        feeder.setShouldRev(false);
        feeder.setLemmeShootBro(false);
        feeder.setShouldAmp(false);
        shooter.stop();
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);

    }
}
