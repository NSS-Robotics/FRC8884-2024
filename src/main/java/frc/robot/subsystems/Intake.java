package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.inner, MotorType.kBrushless);
    private CANSparkMax intakeMotorFollower = new CANSparkMax(Constants.IntakeConstants.outer, MotorType.kBrushless);
    private LaserCan laserCAN = new LaserCan(Constants.LaserCanConstants.laserCan);
    private Shooter m_shooter;

    private SparkPIDController intakePID;
    SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(Constants.IntakeConstants.kS,
            Constants.IntakeConstants.kV,
            Constants.IntakeConstants.kA);

    public void setupMotors() {

        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        intakeMotorFollower.follow(intakeMotor, false);

        intakeMotor.setInverted(true);

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

    public void setupLaserCAN() {
        try {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(10, 10, 8, 8));
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LASERCAN CONFIG FAILED");
        }
    }

    public boolean noteDetected() {
        return (laserCAN.getMeasurement().distance_mm < 10);
    }

    public void setVelocity(double velocity) {
        double arbFF;
        arbFF = intakeFF.calculate(velocity / 60);

        // SmartDashboard.putNumber("Intake arbFF", arbFF);
        // SmartDashboard.putNumber("Intake velocity target", velocity);

        intakePID.setReference(velocity, ControlType.kVelocity, 0, arbFF,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        intakeMotor.set(0);
    }

    public void intake(double speed) {
        if(!noteDetected() || m_shooter.isShooting()) 
            setVelocity(speed);
        else 
            stop();
    }

    public void outtake(double speed) {
        setVelocity(-1 * speed);
    }

    public void setHoldingCurrent() {
        intakeMotor.setVoltage(Constants.IntakeConstants.holdingVoltage);
    }

    public Intake(Shooter m_shooter) {
        setupMotors();
        setupLaserCAN();
        this.m_shooter = m_shooter;
    }
}
