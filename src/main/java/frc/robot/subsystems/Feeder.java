package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Candle;


public class Feeder extends SubsystemBase {
    private Candle candle;
    private CANSparkMax feederMotor = new CANSparkMax(
            Constants.FeederConstants.feederIn,
            MotorType.kBrushless);
    private CANSparkMax feederMotorFollower = new CANSparkMax(
            Constants.FeederConstants.feederOut,
            MotorType.kBrushless);

    private SparkPIDController feederPID;
    SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(
            Constants.FeederConstants.kS,
            Constants.FeederConstants.kV,
            Constants.FeederConstants.kA);
    private LaserCan laserCAN = new LaserCan(
            Constants.LaserCanConstants.laserCan);
    private boolean hasBeenDetected;
    private boolean shouldRev;
    private boolean shouldShoot;
    private boolean lemmeShootBro;
    private boolean shouldAmp;
    private boolean firstShot;
    private boolean isAuto;
    private Shooter shooter;

    public void setupMotors() {

        feederMotorFollower.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults();

        feederMotorFollower.follow(feederMotor, false);

        feederMotor.setInverted(false);

        feederMotor.setSmartCurrentLimit(
                Constants.FeederConstants.currentLimit);
        feederMotorFollower.setSmartCurrentLimit(
                Constants.FeederConstants.currentLimit);

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
                0);

        feederMotor.setOpenLoopRampRate(0.05);

        feederMotor.burnFlash();
    }

    public void setupLaserCAN() {
        try {
            laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCAN.setRegionOfInterest(
                    new LaserCan.RegionOfInterest(0, 0, 12, 12));
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
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void stop() {
        feederMotor.stopMotor();
    }

    public void stopHoldingCurrent() {
        feederMotor.set(0);
    }

    public void intake(double speed) {

        if (shouldShoot || shouldAmp || !hasBeenDetected)  {
            setVelocity(speed);
        } else {
            setVelocity(-0.01);
        }
    }

    public void feed(double speed) {
        if (shooter.isFullSpeed()) {
            setVelocity(speed);
        }
    }

    public void setHasBeenDetected(boolean bool) {
        hasBeenDetected = bool;
    }

    public boolean getHasBeenDetected() {
        return hasBeenDetected;
    }

    public void setShouldRev(boolean bool) {
        shouldRev = bool;
    }

    public boolean getShouldRev() {
        return shouldRev;
    }

    public void setLemmeShootBro(boolean bool) {
        lemmeShootBro = bool;
    }

    public boolean getLemmeShootBro() {
        return lemmeShootBro;
    }

    public void setShouldShoot(boolean bool) {
        shouldShoot = bool;
    }

    public boolean getShouldShoot() {
        return shouldShoot;
    }

    public void setShouldAmp(boolean bool) {
        shouldAmp = bool;
    }

    public boolean getShouldAmp() {
        return shouldAmp;
    }

    public void setFirstShot(boolean bool) {
        firstShot = bool;
    }

    public boolean getFirstShot() {
        return firstShot;
    }

    public void setIsAuto(boolean bool) {
        isAuto = bool;
    }

    public void outtake(double speed) {
        setVelocity(-1 * speed);
    }

    public void setHoldingCurrent() {
        feederMotor.setVoltage(Constants.FeederConstants.holdingVoltage);
    }

    public Feeder(Shooter shooter, Candle candle) {
        setupMotors();
        setupLaserCAN();
        setHasBeenDetected(false);
        setShouldRev(false);
        setLemmeShootBro(false);
        setShouldAmp(false);
        setFirstShot(true);
        this.isAuto = true;
        this.shooter = shooter;
        this.candle = candle;
    }

    @Override
    public void periodic() {
        LaserCan.Measurement Measurement = laserCAN.getMeasurement();
        if (Measurement == null) {
            System.out.println("null");
        } else if (Measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            int distance = Measurement.distance_mm;
            if (distance <= 80 && !shooter.isShooting()) {
                setVelocity(-0.01);
            //} else if (shooter.isShooting()){
                setHasBeenDetected(true);
            }
            // else if (hasBeenDetected && shooter.isFullSpeed()) {
            //     feederMotor.set(Constants.FeederConstants.speed);
            //     hasBeenDetected = false;
            // }
            if (shooter.isFullSpeed()&&!candle.button)  {
            candle.strobe(0,0,255);
            }
            else if(hasBeenDetected&&!candle.button){
                candle.strobe(0,255,0);
            }
            
            
            if (getShouldShoot()) {
                if (getFirstShot())
                    shooter.shoot(4000);
                else
                    shooter.shoot(Constants.ShooterConstants.speed);
            } else if (getShouldAmp()) {
                shooter.shoot(Constants.ShooterConstants.ampspeed);
            } else if (getShouldRev()) {
                if (isAuto) {
                    shooter.shoot(Constants.ShooterConstants.speed);
                } else {
                    shooter.shoot(Constants.ShooterConstants.revSpeed);
                }
            }
            SmartDashboard.putBoolean("Is Detected", hasBeenDetected);
        }
    }
}
