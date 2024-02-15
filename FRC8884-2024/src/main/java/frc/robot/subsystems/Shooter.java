package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter {
    private static TalonFX motor1;
    private static TalonFX motor2;

    public Shooter(int motor1, int motor2) {
        this.motor1 = new TalonFX(motor1);
        this.motor2 = new TalonFX(motor2);
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
