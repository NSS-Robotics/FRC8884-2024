package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;

public class NoteIntake extends Command{
    private final Intake intake;
    private final Feeder feeder;
    
    public NoteIntake(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(intake, feeder);
  }

  @Override
  public void execute() {
    intake.intake(Constants.IntakeConstants.speed);
    feeder.intake(Constants.FeederConstants.speed);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
    System.out.println("Pivot Command Ended");
  }
}
