package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.inner, MotorType.kBrushless);
    private CANSparkMax intakeMotorFollower = new CANSparkMax(Constants.IntakeConstants.outer, MotorType.kBrushless);
    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private RelativeEncoder followerEncoder = intakeMotorFollower.getEncoder();

    private SparkPIDController intakePID;
    SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(Constants.IntakeConstants.kS,
            Constants.IntakeConstants.kV,
            Constants.IntakeConstants.kA);
    
      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                intakeMotor.setVoltage(volts.in(Volts));
                intakeMotorFollower.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("intake-inner")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(intakeEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(intakeEncoder.getVelocity(), RotationsPerSecond));
                log.motor("intake-outer")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            intakeMotorFollower.getAppliedOutput() * intakeMotorFollower.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(followerEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(followerEncoder.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

    public void setupMotors() {

        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        intakeMotorFollower.follow(intakeMotor, true);

        intakeMotor.setInverted(false);

        intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);
        intakeMotorFollower.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);

        intakePID = intakeMotor.getPIDController();

        intakePID.setP(Constants.IntakeConstants.kP, 0);
        intakePID.setI(Constants.IntakeConstants.kI, 0);
        intakePID.setD(Constants.IntakeConstants.kD, 0);
        intakePID.setIZone(0, 0);
        intakePID.setFF(Constants.IntakeConstants.FF, 0);
        intakePID.setOutputRange(Constants.GlobalVariables.outputRangeMin, Constants.GlobalVariables.outputRangeMax, 0);

        intakeMotor.setOpenLoopRampRate(0.05);

        intakeMotor.burnFlash();
}

    public void setVelocity(double velocity) {
        double arbFF;
        arbFF = intakeFF.calculate(velocity / 60);

        // SmartDashboard.putNumber("Intake arbFF", arbFF);
        // SmartDashboard.putNumber("Intake velocity target", velocity);

        intakePID.setReference(velocity, ControlType.kVelocity, 0, arbFF,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
      }
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
      }
    public void stop() {
        intakeMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        intakeMotor.set(0);
    }

    public void intake() {
        setVelocity(750);
    }

    public void setHoldingCurrent() {
        intakeMotor.setVoltage(Constants.IntakeConstants.holdingVoltage);
    }

    public void Outtake() {
        setVelocity(-1 * 500);
    }

    public void cubeShoot(double speed) {
        intakeMotor.set(-1 * speed);
    }

    public Intake() {
        setupMotors();
    }
}
