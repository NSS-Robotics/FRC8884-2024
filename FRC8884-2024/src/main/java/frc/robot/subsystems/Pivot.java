package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private CANSparkMax Lmotor;
    private CANSparkMax Rmotor;
    private RelativeEncoder LmotorEncoder;
    private RelativeEncoder RmotorEncoder;
    private SparkPIDController Lmotorpid;
    private SparkPIDController Rmotorpid;

    public boolean pivotReset = false;

    public void elevatorsetup() {
        // Lmotor Setup
        Lmotor = new CANSparkMax(
                Constants.PivotConstants.leftMotor,
                MotorType.kBrushless);

        Lmotor.restoreFactoryDefaults();
        Lmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Lmotor.setSmartCurrentLimit(30);
        Lmotor.setOpenLoopRampRate(0.5);
        Lmotor.setClosedLoopRampRate(0.5);
        LmotorEncoder = Lmotor.getEncoder();
        Lmotorpid = Lmotor.getPIDController();
        Lmotorpid.setP(Constants.PivotConstants.Kp, 0);
        Lmotorpid.setI(Constants.PivotConstants.Ki, 0);
        Lmotorpid.setD(Constants.PivotConstants.Kd, 0);
        Lmotorpid.setIZone(0);
        Lmotorpid.setOutputRange(-1, 1);

        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        Lmotor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kForward,
                Constants.PivotConstants.MaxRotation);
        Lmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        Lmotor.setCANTimeout(0);

        // Rmotor Setup
        Rmotor = new CANSparkMax(
                Constants.PivotConstants.rightMotor,
                MotorType.kBrushless);

        Rmotor.restoreFactoryDefaults();
        Rmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        Rmotor.setSmartCurrentLimit(30);
        Rmotor.setOpenLoopRampRate(0.5);
        Rmotor.setClosedLoopRampRate(0.5);
        RmotorEncoder = Rmotor.getEncoder();
        Rmotorpid = Rmotor.getPIDController();
        Rmotorpid.setP(Constants.PivotConstants.Kp, 0);
        Rmotorpid.setI(Constants.PivotConstants.Ki, 0);
        Rmotorpid.setD(Constants.PivotConstants.Kd, 0);
        Rmotorpid.setIZone(0);
        Rmotorpid.setOutputRange(-1, 1);

        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        Rmotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        Rmotor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kReverse,
                -Constants.PivotConstants.MaxRotation);
        Rmotor.setCANTimeout(0);

        resetEncoders();

        Lmotor.burnFlash();
        Rmotor.burnFlash();
    }

    public void resetEncoders() {
        LmotorEncoder.setPosition(0);
        RmotorEncoder.setPosition(0);
    }

    public void setPivot(double value) {
        Lmotorpid.setReference(value, CANSparkMax.ControlType.kPosition, 0);
        Rmotorpid.setReference(-value, CANSparkMax.ControlType.kPosition, 0);
    }

    public void stopPivot() {
        Lmotor.set(0);
        Rmotor.set(0);
    }

    public void disablePivotLimits() {
        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }

    public void enablePivotLimits() {
        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        Lmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        Rmotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    public void resetPivot() {
        pivotReset = true;
        LmotorEncoder.setPosition(0);
        RmotorEncoder.setPosition(0);
    }

    public double[] getPivotCurrent() {
        double outputcurrent[] = new double[2];
        outputcurrent[0] = Lmotor.getOutputCurrent();
        outputcurrent[1] = Rmotor.getOutputCurrent();
        return outputcurrent;
    }

    public double[] getPivotEncoder() {
        double outputencoder[] = new double[2];
        outputencoder[0] = LmotorEncoder.getPosition();
        outputencoder[1] = RmotorEncoder.getPosition();
        return outputencoder;
    }

    public void up() {
        Lmotor.set(0.3);
        Rmotor.set(-0.3);
    }

    public void down() {
        Lmotor.set(-0.3);
        Rmotor.set(0.3);
    }

    public Pivot() {
        elevatorsetup();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LmotorEncoder", LmotorEncoder.getPosition());
        SmartDashboard.putNumber("RmotorEncoder", RmotorEncoder.getPosition());
    }
}
