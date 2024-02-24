package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class PivotUp extends Command {

  private final Pivot pivot;

  public PivotUp(Pivot _pivot) {
    pivot = _pivot;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.setPivot(Constants.PivotConstants.PivotAgainstRotations);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("Pivot Command Ended");
  }
}