package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;

public class NoteOuttake extends Command {
    private final Intake intake;
    private final Feeder feeder;
    
    public NoteOuttake(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(intake, feeder);
  }

  @Override
  public void execute() {
    feeder.outtake(Constants.FeederConstants.speed);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    System.out.println("Pivot Command Ended");
  }
}
