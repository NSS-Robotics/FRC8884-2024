package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command{
    private final Shooter shooter;

    public Shoot(Shooter shooter, Candle candle) {
        this.shooter = shooter;
        addRequirements(shooter);
        candle.flow(255,0,1);
      }
    
      @Override
      public void execute() {
        shooter.shoot(Constants.ShooterConstants.speed);

      }
    
      @Override
      public void initialize() {}
    
      @Override
      public void end(boolean interrupted) {
        shooter.stop();

        System.out.println("Pivot Command Ended");
      }
}
