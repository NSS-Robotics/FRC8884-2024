package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Feed extends Command {
    private Feeder feeder;
    private Shooter shooter;

    public Feed(Shooter m_shooter, Feeder feeder) {
        this.feeder = feeder;
        this.shooter = m_shooter;
        addRequirements(feeder, shooter);
    }

    @Override
    public void execute() {
        feeder.feed(Constants.FeederConstants.speed);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
    
}
