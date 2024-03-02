package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Pivot;

public class PivotUp extends Command {

    private final Pivot pivot;
    private final Candle candle;

    public PivotUp(Pivot _pivot, Candle candle) {
        pivot = _pivot;
        this.candle = candle;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double rotations = pivot.getRotations();
        //pivot.getRotations();
        pivot.setPivot(rotations);
        candle.flow(140, 0, 255);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
        candle.setLEDs(170, 247, 250);
    }
}
