package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

    private final Shooter shooter;
    private final Candle candle;

    public Shoot(Shooter shooter, Candle candle) {
        this.shooter = shooter;
        this.candle = candle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.ShooterConstants.speed);
        candle.flow(255, 0, 1);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        candle.setLEDs(170, 247, 250);
    }
}
