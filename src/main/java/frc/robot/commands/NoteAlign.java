package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/** Aims at the target using the limelight TX value (turns tx deg) */
public class NoteAlign extends PIDCommand {

    private Swerve swerve;
    private Limelight limelight;

    public NoteAlign(Swerve swerve, Limelight limelight) {
        super(
            new PIDController(0.04, 0, 0.003),
            limelight::gettx,
            0.0,
            tx -> swerve.turnStates(-tx),
            swerve
        );
        this.swerve = swerve;
        this.limelight = limelight;
        addRequirements(this.swerve, this.limelight);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(2.5);

        System.out.println("Align with Note - Start");
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);
        System.out.println("Align With Note - End");

        super.end(interrupted);
    }
}
