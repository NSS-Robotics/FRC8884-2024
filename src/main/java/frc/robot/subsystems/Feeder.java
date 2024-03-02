package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

    private CANSparkMax feederMotor = new CANSparkMax(
        Constants.FeederConstants.feederIn,
        MotorType.kBrushless
    );
    private CANSparkMax feederMotorFollower = new CANSparkMax(
        Constants.FeederConstants.feederOut,
        MotorType.kBrushless
    );

    private SparkPIDController feederPID;
    SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(
        Constants.FeederConstants.kS,
        Constants.FeederConstants.kV,
        Constants.FeederConstants.kA
    );
    private LaserCan laserCAN = new LaserCan(
        Constants.LaserCanConstants.laserCan
    );
    private boolean hasBeenDetected = false;
    private Shooter shooter;

    public void setupMotors() {
        feederMotorFollower.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults();

        feederMotorFollower.follow(feederMotor, false);

        feederMotor.setInverted(false);

        feederMotor.setSmartCurrentLimit(
            Constants.FeederConstants.currentLimit
        );
        feederMotorFollower.setSmartCurrentLimit(
            Constants.FeederConstants.currentLimit
        );

        feederMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        feederMotorFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

        feederPID = feederMotor.getPIDController();

        feederPID.setP(Constants.FeederConstants.kP, 0);
        feederPID.setI(Constants.FeederConstants.kI, 0);
        feederPID.setD(Constants.FeederConstants.kD, 0);
        feederPID.setIZone(0, 0);
        feederPID.setFF(Constants.FeederConstants.FF, 0);
        feederPID.setOutputRange(
            Constants.GlobalVariables.outputRangeMin,
            Constants.GlobalVariables.outputRangeMax,
            0
        );

        feederMotor.setOpenLoopRampRate(0.05);

        feederMotor.burnFlash();
    }

    public void setupLaserCAN() {
        try {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(
                new LaserCan.RegionOfInterest(0, 0, 12, 12)
            );
            laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LASERCAN CONFIG FAILED");
        }
    }

    public void setVelocity(double velocity) {
        double arbFF;
        arbFF = feederFF.calculate(velocity / 60);

        // SmartDashboard.putNumber("Feeder arbFF", arbFF);
        // SmartDashboard.putNumber("Feeder velocity target", velocity);

        feederPID.setReference(
            velocity,
            ControlType.kVelocity,
            0,
            arbFF,
            SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    public void stop() {
        feederMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        feederMotor.set(0);
    }

    public void intake(double speed) {
        if (!hasBeenDetected) {
            setVelocity(speed);
        } else {
            stop();
        }
    }

    public void setbool(boolean bool) {
        hasBeenDetected = bool;
    }

    public void outtake(double speed) {
        setVelocity(-1 * speed);
    }

    public void setHoldingCurrent() {
        feederMotor.setVoltage(Constants.FeederConstants.holdingVoltage);
    }

    public Feeder(Shooter shooter) {
        setupMotors();
        setupLaserCAN();
        this.shooter = shooter;
    }

    @Override
    public void periodic() {
        LaserCan.Measurement Measurement = laserCAN.getMeasurement();
        if (Measurement == null) {
            System.out.println("null");
        } else if (
            Measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        ) {
            int distance = Measurement.distance_mm;
            if (distance <= 150 && !shooter.isShooting()) {
                hasBeenDetected = true;
                setVelocity(-2);
            } else if (shooter.isShooting()) {
                hasBeenDetected = false;
            }
        }
    }
}
