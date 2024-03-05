package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ClimbPos extends Command {
    private final Pivot pivot;

    public ClimbPos(Pivot _pivot) {
        pivot = _pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setPivot(Constants.PivotConstants.ClimbRotations);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
    }

}
