package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SpeakerShoot extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Feeder feeder;
    private final Candle candle;

    public SpeakerShoot(Shooter _shooter, Pivot _pivot, Feeder _feeder, Candle candle) {
        shooter = _shooter;
        pivot = _pivot;
        feeder = _feeder;
        this.candle = candle;
        addRequirements(shooter, pivot, feeder);
    }

    @Override
    public void execute() {
        double rotations = (Math.max(0, pivot.getRotations()));
        // pivot.setPivot(Constants.PivotConstants.PivotAgainstRotations);
        pivot.setPivot(rotations);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setHasBeenDetected(false);
        candle.ledsOff();
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        shooter.stop();
    }
}
