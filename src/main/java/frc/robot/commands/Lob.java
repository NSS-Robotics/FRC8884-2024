package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Lob extends Command {

    private final Pivot pivot;
    private final Shooter shooter;
    private final Feeder feeder;
    private final double rotations;
    private final Candle candle;

    public Lob(Pivot _pivot, Shooter _shoooter, Feeder _feeder, double _rotations, Candle _candle) {
        pivot = _pivot;
        shooter = _shoooter;
        rotations = _rotations;
        feeder = _feeder;
        candle = _candle;
        addRequirements(pivot, shooter);

    }

    @Override
    public void execute() {
        feeder.setShouldShoot(true);

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
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        shooter.stop();
    }
}
