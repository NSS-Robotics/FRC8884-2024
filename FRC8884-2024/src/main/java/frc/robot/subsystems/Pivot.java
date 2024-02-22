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

    public Pivot() {
        pivotSetup();
    }

    @Override
    public void periodic() {

    }
}
