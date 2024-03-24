package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class NoteIntake extends Command {

    private final Intake intake;
    private final Feeder feeder;
    private final Candle candle;

    public NoteIntake(Intake intake, Feeder feeder, Candle candle) {
        this.intake = intake;
        this.feeder = feeder;
        this.candle = candle;
        addRequirements(intake, feeder);
    }

    @Override
    public void execute() {
        if (feeder.getHasBeenDetected()) {
            feeder.setLemmeShootBro(true);
        }
        
        // if (!feeder.getLemmeShootBro()) {
            intake.intake(Constants.IntakeConstants.speed);
        // }
        feeder.intake(Constants.FeederConstants.speed);

    }

    @Override
    public void initialize() {
        
    }   

    @Override
    public void end(boolean interrupted) {
        if (feeder.getHasBeenDetected()) {
            feeder.setShouldRev(true);
        }  
        intake.stop();
        feeder.stop();
    }
}
