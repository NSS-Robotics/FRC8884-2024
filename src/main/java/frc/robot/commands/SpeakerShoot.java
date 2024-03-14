package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SpeakerShoot extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Candle candle;

    public SpeakerShoot(Shooter _shooter, Pivot _pivot, Candle candle) {
        shooter = _shooter;
        pivot = _pivot;
        this.candle = candle;
        addRequirements(shooter, pivot);
    }

    @Override
    public void execute() {
        double rotations = (Math.max(0, pivot.getRotations()));
        shooter.shoot(Constants.ShooterConstants.speed);
        // pivot.setPivot(Constants.PivotConstants.PivotAgainstRotations);
        pivot.setPivot(rotations);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        shooter.stop();
    }
}
