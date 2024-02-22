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

public class Feeder extends SubsystemBase {

    private CANSparkMax feederMotor = new CANSparkMax(Constants.FeederConstants.feederIn, MotorType.kBrushless);
    private CANSparkMax feederMotorFollower = new CANSparkMax(Constants.FeederConstants.feederOut,
            MotorType.kBrushless);
    private RelativeEncoder feederEncoder = feederMotor.getEncoder();
    private RelativeEncoder followerEncoder = feederMotorFollower.getEncoder();

    private SparkPIDController feederPID;
    SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(Constants.FeederConstants.kS,
            Constants.FeederConstants.kV,
            Constants.FeederConstants.kA);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    (Measure<Voltage> volts) -> {
                        feederMotor.setVoltage(volts.in(Volts));
                        feederMotorFollower.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("feeder-inner")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                feederMotor.getAppliedOutput() * feederMotor.getBusVoltage(), Volts))
                                .angularPosition(m_angle.mut_replace(feederEncoder.getPosition(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(feederEncoder.getVelocity(), RotationsPerSecond));
                        log.motor("feeder-outer")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                feederMotorFollower.getAppliedOutput()
                                                        * feederMotorFollower.getBusVoltage(),
                                                Volts))
                                .angularPosition(m_angle.mut_replace(followerEncoder.getPosition(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(followerEncoder.getVelocity(), RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("shooter")
                    this));

    public void setupMotors() {

        feederMotorFollower.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults();

        feederMotorFollower.follow(feederMotor, false);

        feederMotor.setInverted(false);

        feederMotor.setSmartCurrentLimit(Constants.FeederConstants.currentLimit);
        feederMotorFollower.setSmartCurrentLimit(Constants.FeederConstants.currentLimit);

        feederPID = feederMotor.getPIDController();

        feederPID.setP(Constants.FeederConstants.kP, 0);
        feederPID.setI(Constants.FeederConstants.kI, 0);
        feederPID.setD(Constants.FeederConstants.kD, 0);
        feederPID.setIZone(0, 0);
        feederPID.setFF(Constants.FeederConstants.FF, 0);
        feederPID.setOutputRange(Constants.GlobalVariables.outputRangeMin, Constants.GlobalVariables.outputRangeMax, 0);

        feederMotor.setOpenLoopRampRate(0.05);

        feederMotor.burnFlash();
    }

    public void setVelocity(double velocity) {
        double arbFF;
        arbFF = feederFF.calculate(velocity / 60);

        // SmartDashboard.putNumber("Feeder arbFF", arbFF);
        // SmartDashboard.putNumber("Feeder velocity target", velocity);

        feederPID.setReference(velocity, ControlType.kVelocity, 0, arbFF,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public void stop() {
        feederMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        feederMotor.set(0);
    }

    public void feeder() {
        setVelocity(250);
    }
    
    public void outtake() {
        setVelocity(-1 * 250);
    }

    public void setHoldingCurrent() {
        feederMotor.setVoltage(Constants.FeederConstants.holdingVoltage);
    }

    public Feeder() {
        setupMotors();
    }
}
