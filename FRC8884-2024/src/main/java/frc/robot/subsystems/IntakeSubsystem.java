package frc.robot.subsystems;

public class Intake {
    package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ONE, MotorType.kBrushless);
    private CANSparkMax intakeMotorFollower = new CANSparkMax(Constants.INTAKE_MOTOR_TWO, MotorType.kBrushless);

    private SparkMaxPIDController intakePID;
    SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(Constants.INTAKE_KS, Constants.INTAKE_KV,
        Constants.INTAKE_KA);

    DigitalInput limitSwitch = new DigitalInput(1);
    
    public void setupMotors(){
      
        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        intakeMotorFollower.follow(intakeMotor, true);

        intakeMotor.setInverted(false);

        intakeMotor.setSmartCurrentLimit(Constants.CURRENT_LIMIT_INTAKE);
        intakeMotorFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT_INTAKE);

        intakePID = intakeMotor.getPIDController();

        intakePID.setP(Constants.INTAKE_PID0_P, 0);
        intakePID.setI(Constants.INTAKE_PID0_I, 0);
        intakePID.setD(Constants.INTAKE_PID0_D, 0);
        intakePID.setIZone(0, 0);
        intakePID.setFF(Constants.INTAKE_PID0_F, 0);
        intakePID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 0);
    
        intakePID.setP(Constants.INTAKE_PID1_P, 1);
        intakePID.setI(Constants.INTAKE_PID1_I, 1);
        intakePID.setD(Constants.INTAKE_PID1_D, 1);
        intakePID.setIZone(0, 1);
        intakePID.setFF(Constants.INTAKE_PID1_F, 1);
        intakePID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, 1);
    
        intakeMotor.setOpenLoopRampRate(0.05);
        
        intakeMotor.burnFlash();
        intakeMotorFollower.burnFlash();
    }

    public void setVelocity(double velocity) {
      double arbFF;
      arbFF = intakeFF.calculate(velocity/60);
  
      //SmartDashboard.putNumber("Intake arbFF", arbFF);
      //SmartDashboard.putNumber("Intake velocity target", velocity);
  
      intakePID.setReference(velocity, ControlType.kVelocity, Constants.INTAKE_PID_SLOT_VELOCITY, arbFF,
          SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void stop(){
            intakeMotor.stopMotor();
          }
    
    public void stopHoldingCurrent(){
            intakeMotor.set(0);
        }

    public void cubeIntake(double speed){
        setVelocity(speed);
      }

    public void setHoldingCurrent(){
      intakeMotor.setVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE);
      }

    public void cubeOuttake(double speed){
        setVelocity(-1*speed);
      }
    public void cubeShoot(double speed){
      intakeMotor.set(-1*speed);
    }
    
    public void holdingCube(){
      boolean state = limitSwitch.get(); //get the state of the switch
      SmartDashboard.putBoolean("Holding Cube", state);
    }


    public IntakeSubsystem() {
        setupMotors();
      }
}
}
