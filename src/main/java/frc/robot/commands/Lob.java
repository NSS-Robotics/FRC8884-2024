package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Lob extends Command {

    private final Pivot pivot;
    private final Shooter shooter;
    private final double rotations;

    public Lob(Pivot _pivot, Shooter _shoooter, double _rotations) {
        pivot = _pivot;
        shooter = _shoooter;
        rotations =  _rotations;
        addRequirements(pivot, shooter);

    }

    @Override
    public void execute() {
        if (pivot.iwannadie()) {
            pivot.killme(false);
            pivot.kys(true);
        }

        pivot.setPivot(rotations);
        shooter.shoot(Constants.ShooterConstants.speed);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        pivot.hasBeenDetected(false);
        pivot.killme(false);
        pivot.kys(false);
        pivot.shouldShoot(false);
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        shooter.stop();
    }
}
