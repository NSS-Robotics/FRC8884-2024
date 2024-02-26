package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {
    private static TalonFX shooterMotor = new TalonFX(20);
    private static TalonFX shooterFollower = new TalonFX(21);
    private static VelocityVoltage shooterVelocityVoltage;
    private static VelocityVoltage followerVelocityVoltage;

    public Shooter() {
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.ShooterConstants.kS; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = Constants.ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = Constants.ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = Constants.ShooterConstants.kI;
        slot0Configs.kD = Constants.ShooterConstants.kD;

        var slot1Configs = new Slot0Configs();
        slot1Configs.kS = Constants.ShooterConstants.kS; // Add 0.05 V output to overcome static friction
        slot1Configs.kV = Constants.ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = Constants.ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = Constants.ShooterConstants.kI;
        slot1Configs.kD = Constants.ShooterConstants.kD;

        shooterMotor.getConfigurator().apply(slot0Configs);

        shooterFollower.getConfigurator().apply(slot1Configs);   
    }

    public void setVelocity(double velocity) {
        double desiredrps = velocity/60;
        double mep = velocity/-60; //should be 130 for spin
        shooterVelocityVoltage = new VelocityVoltage(desiredrps);
        followerVelocityVoltage = new VelocityVoltage(mep);
        shooterMotor.setControl(shooterVelocityVoltage);
        shooterFollower.setControl(followerVelocityVoltage);
    }

    public void shoot(double speed) {
        setVelocity(speed);
    }

    public void stop() {
        setVelocity(0);
    }
}
