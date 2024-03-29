package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SpinShooter extends Command {

    private final Shooter shooter;

    public SpinShooter(Shooter _shooter) {
        shooter = _shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(1000);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
}
