package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Pivot extends SubsystemBase {

    private static TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotor);
    private static TalonFX pivotFollower = new TalonFX(Constants.PivotConstants.followerMotor);
    private static PositionVoltage pivotPositionVoltage;
    private CANcoder encoder = new CANcoder(13);

    private SparkPIDController pivotPID;
    private Swerve s_swerve;
    private double yInt;
    private double amp;

    public boolean pivotReset = false;

    public void pivotSetup() {
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = Constants.PivotConstants.kS;
        slot0Configs.kP = Constants.PivotConstants.kV;
        slot0Configs.kP = Constants.PivotConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = Constants.PivotConstants.kI;
        slot0Configs.kD = Constants.PivotConstants.kD;

        

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setPivot(double position) {
        pivotPositionVoltage = new PositionVoltage(position);

        pivotMotor.setControl(pivotPositionVoltage);
        pivotFollower.setControl(new Follower(Constants.PivotConstants.pivotMotor, true));
        
    }

    public void setClimb(double position) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition, 1);
    }

    public double getRotations() {
        double distance = 0;

        double[] dist = s_swerve.getSpeakerDistances();
        distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        double rotations = yInt -
                46.1 * distance +
                16.2 * Math.pow(distance, 2) -
                2.81 * Math.pow(distance, 3) +
                0.194 * Math.pow(distance, 4);

        // System.out.println("distance: " + distance);
        // System.out.println("rotations: " + rotations);
        // System.out.println(
        // "rotations off: " + (rotations - pivotEncoder.getPosition())
        // );
        return rotations;
    }

    public double encoderPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    public void printPivotData() {
        double[] dist = s_swerve.getSpeakerDistances();
        double distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        System.out.println("Dto speaker: " + distance);
        System.out.println("Pivot pos:   " + encoderPosition());
        System.out.println("shoot rot:   " + getRotations());
        System.out.println("diff:" + (getRotations() - encoderPosition()));
        // System.out.println("rot: " + Constants.PivotConstants.PivotAgainstRotations);
    }

    public Pivot(Swerve swerve) {
        yInt = 87;
        amp = Constants.PivotConstants.AmpRotations;

        pivotSetup();
        s_swerve = swerve;
    }

    public void setYInt(double x) {
        yInt = x;
    }

    public double getYInt() {
        return yInt;
    }

    public void setAmp(double x) {
        amp = x;
    }

    public double getAmp() {
        return amp;
    }

    public void changeYInt(double x) {
        yInt += x;
    }

    @Override
    public void periodic() {
        // s_swerve.printPosData();
        // printPivotData();
        double[] dist = s_swerve.getSpeakerDistances();
        double distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        SmartDashboard.putNumber("Y-Int", yInt);
        SmartDashboard.putNumber("Amp rotations", amp);
        SmartDashboard.putNumber("rotations", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("exp rot", getRotations());
        SmartDashboard.putNumber("dto speaker", distance);

    }
}
