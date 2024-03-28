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
        // if (feeder.getHasBeenDetected()) {
        //     feeder.setLemmeShootBro(true);
        // }
        feeder.setIsIntaking(true);
        
        // if (feeder.getLemmeShootBro()) {
        if (feeder.getShouldShoot()) {
            feeder.intake(Constants.FeederConstants.feedSpeed);
        } else if (feeder.getShouldAmp()) {
            feeder.intake(Constants.FeederConstants.speed);
        } else {
            feeder.intake(Constants.FeederConstants.speed);
            if (!feeder.getHasBeenDetected()) {
            intake.intake(Constants.IntakeConstants.speed);
            }
        }
    }

    @Override
    public void initialize() {
        
    }   

    @Override
    public void end(boolean interrupted) {
        feeder.setIsIntaking(false);
        intake.stop();
        feeder.stop();
    }
}
