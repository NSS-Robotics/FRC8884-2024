package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ClimbPos extends Command {
    private final Pivot pivot;
    private final Shooter shooter;

    public ClimbPos(Pivot _pivot, Shooter _shooter) {
        shooter = _shooter;
        pivot = _pivot;
        addRequirements(pivot, shooter);
    }

    @Override
    public void execute() {
        pivot.setPivot(Constants.PivotConstants.TrapAgainstRotations);
        shooter.shoot(Constants.ShooterConstants.speed);
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
