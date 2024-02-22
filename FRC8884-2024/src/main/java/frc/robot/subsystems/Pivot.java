package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Pivot extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private CANSparkMax pivotFollower;
    private CANcoder Encoder;
    private SparkPIDController pivotPID;
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
                        pivotMotor.setVoltage(volts.in(Volts));
                        pivotFollower.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("left-pivot")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage(), Volts))
                                .angularPosition(
                                        m_angle.mut_replace(Encoder.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(Encoder.getVelocity().getValueAsDouble(),
                                                RotationsPerSecond));
                        log.motor("right-pivot")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                pivotFollower.getAppliedOutput()
                                                        * pivotFollower.getBusVoltage(),
                                                Volts))
                                .angularPosition(
                                        m_angle.mut_replace(Encoder.getPosition().getValueAsDouble(), Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(Encoder.getVelocity().getValueAsDouble(),
                                                RotationsPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("shooter")
                    this));

    public boolean pivotReset = false;

    public void pivotSetup() {
        pivotMotor = new CANSparkMax(Constants.PivotConstants.pivotMotor, MotorType.kBrushless);
        pivotFollower = new CANSparkMax(Constants.PivotConstants.followerMotor, MotorType.kBrushless);
        Encoder = new CANcoder(13);
        pivotMotor.restoreFactoryDefaults();
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.follow(pivotMotor, true);

        pivotMotor.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);
        pivotFollower.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.kP, 0);
        pivotPID.setI(Constants.PivotConstants.kI, 0);
        pivotPID.setD(Constants.PivotConstants.kD, 0);
        pivotPID.setIZone(0, 0);
        pivotPID.setOutputRange(Constants.GlobalVariables.outputRangeMin, Constants.GlobalVariables.outputRangeMax, 0);

    }

    public void resetEncoders() {
        Encoder.setPosition(0);
    }

    public void setPivot(double value) {
        pivotPID.setReference(value, CANSparkBase.ControlType.kPosition, 0);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Pivot() {
        pivotSetup();
    }

    @Override
    public void periodic() {

    }
}
