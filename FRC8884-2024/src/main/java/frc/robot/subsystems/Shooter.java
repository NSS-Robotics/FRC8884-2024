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
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {

    private static TalonFX shooterMotor = new TalonFX(20);
    private static TalonFX shooterFollower = new TalonFX(21);
    private static VelocityVoltage shooterVelocityVoltage;

    public Shooter() {

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.ShooterConstants.kS; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = Constants.ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = Constants.ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = Constants.ShooterConstants.kI;
        slot0Configs.kD = Constants.ShooterConstants.kD;

        shooterMotor.getConfigurator().apply(slot0Configs);

        shooterFollower.setControl(new Follower(shooterMotor.getDeviceID(), true));      
    }

    public void setVelocity(double velocity) {
        double desiredrps = velocity/60;
        shooterVelocityVoltage = new VelocityVoltage(desiredrps);
        shooterMotor.setControl(shooterVelocityVoltage);
    }

    public void shoot(double speed)
    {
        setVelocity(speed);
    }

    public void stop()
    {
        setVelocity(0);
    }

    
}
