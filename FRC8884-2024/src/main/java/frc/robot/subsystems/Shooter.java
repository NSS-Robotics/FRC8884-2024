package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    private static TalonFX shooterMotor = new TalonFX(20);
    private static TalonFX shooterFollower = new TalonFX(21);
    private static VelocityVoltage shooterVelocityVoltage;

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
                    shooterMotor.setVoltage(volts.in(Volts));
                    shooterFollower.setVoltage(volts.in(Volts));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism
                // being
                // characterized.
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("left-shooter")
                            .voltage(
                                    m_appliedVoltage.mut_replace(
                                            shooterMotor.getDutyCycle().getValueAsDouble() * shooterMotor.getSupplyVoltage().getValueAsDouble(), Volts))
                            .angularPosition(
                                    m_angle.mut_replace(shooterMotor.getPosition().getValueAsDouble(), Rotations))
                            .angularVelocity(
                                    m_velocity.mut_replace(shooterMotor.getVelocity().getValueAsDouble(),
                                            RotationsPerSecond));
                    log.motor("right-shooter")
                            .voltage(
                                    m_appliedVoltage.mut_replace(
                                            shooterFollower.getDutyCycle().getValueAsDouble()
                                                    * shooterFollower.getSupplyVoltage().getValueAsDouble(),
                                            Volts))
                            .angularPosition(
                                    m_angle.mut_replace(shooterFollower.getPosition().getValueAsDouble(), Rotations))
                            .angularVelocity(
                                    m_velocity.mut_replace(shooterFollower.getVelocity().getValueAsDouble(),
                                            RotationsPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test
                // state in
                // WPILog with this subsystem's name ("shooter")
                this));

    public Shooter() {

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        shooterMotor.getConfigurator().apply(slot0Configs);

        shooterFollower.setControl(new Follower(shooterMotor.getDeviceID(), true));      
    }

    public void setVelocity(double velocity) {
        double desiredrps = velocity/60;
        shooterVelocityVoltage = new VelocityVoltage(velocity);
        shooterMotor.setControl(shooterVelocityVoltage);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
}
