package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Feeder extends SubsystemBase {

    private CANSparkMax feederMotor = new CANSparkMax(Constants.FeederConstants.feederIn, MotorType.kBrushless);
    private CANSparkMax feederMotorFollower = new CANSparkMax(Constants.FeederConstants.feederOut,
            MotorType.kBrushless);

    private SparkPIDController feederPID;
    SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(Constants.FeederConstants.kS,
            Constants.FeederConstants.kV,
            Constants.FeederConstants.kA);


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

    public void stop() {
        feederMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        feederMotor.set(0);
    }

    public void intake(double speed) {
        setVelocity(speed);
    }
    
    public void outtake(double speed) {
        setVelocity(-1 * speed);
    }

    public void setHoldingCurrent() {
        feederMotor.setVoltage(Constants.FeederConstants.holdingVoltage);
    }

    public Feeder() {
        setupMotors();
    }
}
