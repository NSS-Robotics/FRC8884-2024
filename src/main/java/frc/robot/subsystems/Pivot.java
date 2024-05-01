package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    private static TalonFX pivotMotor = new TalonFX(Constants.PivotConstants.pivotMotor);
    private static TalonFX pivotFollower = new TalonFX(Constants.PivotConstants.followerMotor);
    private static TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = talonFXConfig.Slot0;
    private static Slot1Configs slot1Configs = talonFXConfig.Slot1;
    private static PositionVoltage pivotPositionVoltage;
    private CANcoder encoder = new CANcoder(13);

    private SparkPIDController pivotPID;
    private Feeder feeder;
    private Swerve s_swerve;
    private double yInt;
    private double amp;
    private double testRotations;
    public boolean pivotReset = false;

    public void pivotSetup() {
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

        CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = 0.134521484375; // 0.376708984375 + 0.29296875;
        // 0.091796875 + 0.01953125
        encoder.getConfigurator().apply(CANcoderConfig);

        slot0Configs.kP = Constants.PivotConstants.kS;
        slot0Configs.kP = Constants.PivotConstants.kV;
        slot0Configs.kP = Constants.PivotConstants.kP;
        slot0Configs.kI = Constants.PivotConstants.kI;
        slot0Configs.kD = Constants.PivotConstants.kD;

        slot1Configs.kP = Constants.PivotConstants.kV;
        slot1Configs.kP = Constants.PivotConstants.kS;
        slot1Configs.kP = Constants.PivotConstants.climbP;
        slot1Configs.kI = Constants.PivotConstants.climbI;
        slot1Configs.kD = Constants.PivotConstants.climbD;

        talonFXConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotFollower.setNeutralMode(NeutralModeValue.Brake);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setPivot(double position) {
        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotPositionVoltage = new PositionVoltage(position);

        pivotMotor.setControl(pivotPositionVoltage);
        pivotFollower.setControl(new Follower(Constants.PivotConstants.pivotMotor, true));
    }

    public void setClimb(double position) {
        pivotMotor.getConfigurator().apply(slot1Configs);
        pivotPositionVoltage = new PositionVoltage(position);

        pivotMotor.setControl(pivotPositionVoltage);
        pivotFollower.setControl(new Follower(Constants.PivotConstants.pivotMotor, true));
    }

    public double getRotations() {
        double distance = 0;

        double[] dist = s_swerve.getSpeakerDistances();
        distance = Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
        double rotations = yInt -
                0.382 * distance +
                0.0829 * Math.pow(distance, 2) -
                0.0057 * Math.pow(distance, 3);

        // System.out.println("distance: " + distance);
        // System.out.println("rotations: " + rotations);
        // System.out.println(
        // "rotations off: " + (rotations - pivotEncoder.getPosition())
        // );
        return rotations;
        // return testRotations;
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

    public Pivot(Swerve swerve, Feeder feeder) {
        yInt = 1.01;
        amp = Constants.PivotConstants.AmpRotations;
        SmartDashboard.putNumber("Amp Rot", amp);
        testRotations = 0.4;
        SmartDashboard.putNumber("Test Rot", testRotations);

        pivotSetup();
        s_swerve = swerve;
        this.feeder = feeder;
    }

    public void setYInt(double x) {
        yInt = x;
    }

    public boolean iwannadie() {
        return feeder.getShouldRev();
    }

    public void killme(boolean kill) {
        feeder.setShouldRev(kill);
    }

    public void kys(boolean k) {
        feeder.setLemmeShootBro(k);
    }

    public void hasBeenDetected(boolean bool) {
        feeder.setHasBeenDetected(bool);
    }

    public void shouldShoot(boolean bool) {
        feeder.setShouldRev(bool);
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

    public void changeAmp(double x) {
        amp += x;
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
        SmartDashboard.putNumber("pivot cancoder", encoder.getPosition().getValueAsDouble());
        testRotations = SmartDashboard.getNumber("Test Rot", 0.25);
        amp = SmartDashboard.getNumber("Amp Rot", 1.5);
    }
}
