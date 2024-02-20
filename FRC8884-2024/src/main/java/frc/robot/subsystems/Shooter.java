package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants;

public class Shooter extends TimedRobot {
    
    private static TalonFX motor1 = new TalonFX(20);
    private static TalonFX motor2 = new TalonFX(21);


    public void Shooter() {
        
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        motor1.getConfigurator().apply(slot0Configs);

        // in init function, set slot 0 gains
        var slot1Configs = new Slot0Configs();
        slot1Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
        slot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0; // no output for error derivative

        motor2.getConfigurator().apply(slot1Configs);
        
    }

    public static void in() {
        motor1.set(-1);
        motor2.set(1);
    }

    public static void out() {
        motor1.set(1);
        motor2.set(-1);
    }

    public static void stop() {
        motor1.set(0);
        motor2.set(0);
    }
}
