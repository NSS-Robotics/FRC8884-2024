package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends Command{
    private final Shooter shooter;
    private final Candle candle;

    public AmpShoot(Shooter shooter, Candle candle) {
        this.shooter = shooter;
        this.candle = candle;
        addRequirements(shooter);
      }
    
      @Override
      public void execute() {
        shooter.shoot(Constants.ShooterConstants.ampspeed);
        candle.flow(255, 0, 0);
      }
    
      @Override
      public void initialize() {}
    
      @Override
      public void end(boolean interrupted) {
        shooter.stop();
        candle.setLEDs(170, 247, 250);
      }
}
