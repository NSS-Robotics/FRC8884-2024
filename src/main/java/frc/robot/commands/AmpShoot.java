package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AmpShoot extends Command {

    private final Shooter shooter;
    private final Pivot pivot;
    private final Candle candle;

    public AmpShoot(Shooter shooter, Pivot pivot, Candle candle) {
        this.shooter = shooter;
        this.pivot = pivot;
        this.candle = candle;
        addRequirements(shooter, pivot);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.ShooterConstants.ampspeed);
        pivot.setPivot(Constants.PivotConstants.AmpRotations);
        
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);

    }
}
