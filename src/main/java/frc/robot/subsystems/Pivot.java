package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        pivotMotor =
            new CANSparkMax(
                Constants.PivotConstants.pivotMotor,
                MotorType.kBrushless
            );
        pivotFollower =
            new CANSparkMax(
                Constants.PivotConstants.followerMotor,
                MotorType.kBrushless
            );
        pivotEncoder = pivotMotor.getEncoder();
        followerEncoder = pivotFollower.getEncoder();
        Encoder = new CANcoder(13);
        pivotMotor.restoreFactoryDefaults();
        pivotFollower.restoreFactoryDefaults();
        pivotFollower.follow(pivotMotor, true);

        pivotMotor.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);
        pivotFollower.setSmartCurrentLimit(
            Constants.IntakeConstants.currentLimit
        );

        pivotPID = pivotMotor.getPIDController();

        pivotPID.setP(Constants.PivotConstants.kP, 0);
        pivotPID.setI(Constants.PivotConstants.kI, 0);
        pivotPID.setD(Constants.PivotConstants.kD, 0);
        pivotPID.setIZone(0, 0);
        pivotPID.setOutputRange(
            Constants.GlobalVariables.outputRangeMin,
            Constants.GlobalVariables.outputRangeMax,
            0
        );
        
        pivotPID.setP(Constants.PivotConstants.climbP, 1);
        pivotPID.setI(Constants.PivotConstants.climbI, 1);
        pivotPID.setD(Constants.PivotConstants.climbD, 1);
        pivotPID.setIZone(0, 1);
        pivotPID.setOutputRange(
            Constants.GlobalVariables.outputRangeMin,
            Constants.GlobalVariables.outputRangeMax,
            1
        );
    }

    public void resetEncoders() {
        Encoder.setPosition(0);
        pivotEncoder.setPosition(0);
        followerEncoder.setPosition(0);
    }

    public void setPivot(double position) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition, 0);
    }

    public void climb(double position) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition, 1);
    }

    public double getRotations() {
        double distance = 0;

        double[] dist = s_swerve.getSpeakerDistances();
        distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        double rotations =
            24.5 +
            2.94 *
            distance -
            4.73 *
            Math.pow(distance, 2) +
            1.29 *
            Math.pow(distance, 3) -
            0.109 *
            Math.pow(distance, 4);
        System.out.println("rotations: " + rotations);
        System.out.println(
            "rotations off: " + (rotations - pivotEncoder.getPosition())
        );
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
        //System.out.println(pivotEncoder.getPosition());
        //System.out.println(distance);
    }
}
