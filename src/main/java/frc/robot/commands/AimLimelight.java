package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.Optional;
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

  public AimLimelight(Swerve _swerve, Limelight _limelight) {
    super(
      new PIDController(0.12, 0, 0.1),
      _swerve::getLimelightrz,
      getAngle(_swerve),
      tx -> _swerve.turnStates(-tx),
      _swerve
    );
    System.out.println(getAngle(_swerve));
    System.out.println(_swerve.getLimelightrz());
    limelight = _limelight;
    swerve = _swerve;
    addRequirements(swerve, limelight);
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(0);
    System.out.println("Align With Limelight - Start");
  }

  public static double getAngle(Swerve swerve) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double[] dist = swerve.getDSpeaker();
    double v = Math.toDegrees(Math.atan(dist[1]/dist[0]));

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      v += 180;
    }
    return v;
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, false, false);
    System.out.println("Align With Limelight - End");
    //limelight.setPipeline(1);
    super.end(interrupted);
  }
}
