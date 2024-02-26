package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AmpShoot extends Command{
    private final Shooter shooter;

    public AmpShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
      }
    
      @Override
      public void execute() {
        shooter.shoot(Constants.ShooterConstants.ampspeed);

      }
    
      @Override
      public void initialize() {}
    
      @Override
      public void end(boolean interrupted) {
        shooter.stop();
        

        System.out.println("Pivot Command Ended");
      }
}
