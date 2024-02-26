package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Feeder;

public class NoteOuttake extends Command {
    public final Intake intake;
    public final Feeder feeder;
    public final Candle candle;

    public NoteOuttake(Intake intake, Feeder feeder, Candle candle) {
        this.intake = intake;
        this.feeder = feeder;
        this.candle = candle;
        addRequirements(intake, feeder);
    }

    @Override
    public void execute() {
        feeder.outtake(Constants.FeederConstants.speed);
        candle.flow(255,140,0);
    }

    @Override
    public void initialize() {}

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        candle.setLEDs(170, 247, 250);
        System.out.println("Pivot Command Ended");
    }
}
