package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Pivot;

public class PivotUp extends Command {

  private final Pivot pivot;

  public PivotUp(Pivot _pivot, Candle candle) {
    pivot = _pivot;
    addRequirements(pivot);
    candle.flow(140, 0, 255);
  }

  @Override
  public void execute() {
    pivot.setPivot(pivot.getRotations());
    //pivot.setPivot(Constants.PivotConstants.AmpRotations);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    pivot.setPivot(Constants.PivotConstants.PivotIntakeRotation);
    System.out.println("Pivot Command Ended");
  }
}