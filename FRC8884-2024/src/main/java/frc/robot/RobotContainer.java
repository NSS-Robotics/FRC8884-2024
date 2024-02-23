// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.TestAuto;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Driver Controller */

    private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
    // private final Swerve m_exampleSubsystem = new Swerve();
    JoystickButton a = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    JoystickButton y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton b = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    JoystickButton x = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    JoystickButton rb = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    JoystickButton lb = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    // Replace with CommandPS4Controller or CommandJoystick if needed
    
    


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
        private final JoystickButton robotCentric = new JoystickButton(m_driverController,
            XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_swerve = new Swerve();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake();
    public final Pivot m_pivot = new Pivot();
    public final Feeder m_feeder = new Feeder();
    public final Candle l_candle = new Candle();

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final TestAuto testAuto = new TestAuto(
        // "TestPath",
        s_swerve,
        () -> false
    );


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        l_candle.rainbow();
        s_swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_swerve,
                        () -> m_driverController.getRawAxis(translationAxis),
                        () -> m_driverController.getRawAxis(strafeAxis),
                        () -> -m_driverController.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the trigger bindings
        configureBindings();

        m_chooser.addOption("TestAuto", testAuto.followTrajectory());

        SmartDashboard.putData("Auto mode", m_chooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        /* Driver Buttons */

        // zeroGyro.whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // a.whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // x.whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // b.whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        
        x.whileTrue(new NoteIntake(m_intake, m_feeder));
        lb.whileTrue(new NoteOuttake(m_intake, m_feeder));
        a.whileTrue(new IntakePos(m_pivot));
        b.whileTrue(new PivotUp(m_pivot));
        rb.whileTrue(new Shoot(m_shooter));
        rb.whileTrue(new InstantCommand(l_candle::twinkle));
        rb.whileFalse(new InstantCommand(l_candle::rainbow));
        zeroGyro.whileTrue(new InstantCommand(s_swerve::zeroGyro));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    // An example command will be run in autonomous
        System.out.println("AUTO");

        return m_chooser.getSelected();
}
}
