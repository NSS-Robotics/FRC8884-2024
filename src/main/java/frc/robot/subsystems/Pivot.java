package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;

public class Pivot extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private CANSparkMax pivotFollower;
    private RelativeEncoder pivotEncoder;
    private RelativeEncoder followerEncoder;
    private CANcoder Encoder;
    private SparkPIDController pivotPID;
    private Swerve s_swerve;

    public boolean pivotReset = false;

    public void pivotSetup() {
        pivotMotor = new CANSparkMax(Constants.PivotConstants.pivotMotor, MotorType.kBrushless);
        pivotFollower = new CANSparkMax(Constants.PivotConstants.followerMotor, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        followerEncoder = pivotFollower.getEncoder();
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
        pivotEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    public void setPivot(double position) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition, 0);
    }

    public double getRotations() {
        double distance = 0;

        double[] dist = s_swerve.getSpeakerDistances();
        distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        double rotations = 24.6 * Math.pow(distance, -0.340);
        Pose2d pose = s_swerve.getLimelightBotPose();

        // System.out.println("Pos X: " + pose.getX());
        // System.out.println("Pos Y: " + pose.getY());
        // System.out.println("Distance: " + distance);
        // System.out.println("Rotations: " + rotations + '\n');
        return rotations;
    }

    public Pivot(Swerve swerve) {
        pivotSetup();
        s_swerve = swerve;
    }
 
    @Override
    public void periodic() {
        double[] dist = s_swerve.getSpeakerDistances();
        double distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        System.out.println(distance);
    }
}
