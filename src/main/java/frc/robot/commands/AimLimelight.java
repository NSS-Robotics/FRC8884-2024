package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** Aims at the target using the limelight TX value (turns tx deg) */
public class AimLimelight extends PIDCommand {

  private Swerve swerve;
  private Limelight limelight;

  public AimLimelight(
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpoint,
    DoubleConsumer useOutput,
    Subsystem[] requirements
  ) {
    super(controller, measurementSource, setpoint, useOutput, requirements);
  }

  public AimLimelight(Swerve _swerve, Limelight limelight) {
    super(
      new PIDController(0.12, 0.0, 0.0),
      limelight::gettx,
      0.0,
      tx -> _swerve.turnStates(-tx),
      _swerve
    );
    swerve = _swerve;
    addRequirements(swerve, limelight);
    getController().enableContinuousInput(-180, 180);
    System.out.println("Align With Limelight - Start");
    this.limelight = limelight;
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, false, false);
    System.out.println("Align With Limelight - End");
    System.out.println(limelight.gettx());

    super.end(interrupted);
  }
}
