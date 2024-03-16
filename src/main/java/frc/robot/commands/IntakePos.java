package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class IntakePos extends Command {

    private final Pivot pivot;

    public IntakePos(Pivot _pivot) {
        pivot = _pivot;
        int veryimprotantint = 1;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setClimb(Constants.PivotConstants.PivotDownRotation);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
    }
}
