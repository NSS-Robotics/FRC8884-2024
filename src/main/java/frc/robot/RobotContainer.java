// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.FourPiece;
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
    JoystickButton jig = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    // Replace with CommandPS4Controller or CommandJoystick if needed

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);

    /* Subsystems */
    public final Candle l_candle = new Candle();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake(m_shooter);
    public final Feeder m_feeder = new Feeder(m_shooter);
    public final Limelight l_limelight_april = new Limelight("april");
    public final Limelight l_limelight_intake = new Limelight("intake");
    public final Swerve s_swerve = new Swerve(l_limelight_april);
    public final Pivot m_pivot = new Pivot(s_swerve);
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final FourPiece fourPiece = new FourPiece(
            "FourPiece",
            4,
            m_feeder,
            m_intake,
            l_limelight_april,
            m_pivot,
            m_shooter,
            s_swerve,
            l_candle,
            () -> false);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        l_candle.setLEDs(170, 247, 250);
        s_swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_swerve,
                        () -> m_driverController.getRawAxis(translationAxis),
                        () -> m_driverController.getRawAxis(strafeAxis),
                        () -> -m_driverController.getRawAxis(rotationAxis) * 0.75,
                        () -> false));
        l_limelight_april.setPipeline(0);

        // Configure the trigger bindings
        configureBindings();

        m_chooser.addOption("FourPiece", fourPiece.followTrajectory());

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

        x.whileTrue(new NoteOuttake(m_intake, m_feeder, l_candle));
        lb.whileTrue(new NoteIntake(m_intake, m_feeder, l_candle));
        a.whileTrue(new AimLimelight(s_swerve, l_limelight_april));
        b.whileTrue(new Jiggle(m_pivot));
        b.whileTrue(new AmpShoot(m_shooter, l_candle));
        rb.whileTrue(new PivotUp(m_pivot, l_candle));
        rb.whileTrue(new Shoot(m_shooter, l_candle));
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