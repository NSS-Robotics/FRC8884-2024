// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
<<<<<<< HEAD
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
=======
import frc.robot.commands.TeleopSwerve;
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
<<<<<<< HEAD
// import frc.robot.subsystems.Swerve;

=======
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.PS4Controller;
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901
import edu.wpi.first.wpilibj.XboxController;
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
<<<<<<< HEAD
    // private final Swerve m_exampleSubsystem = new Swerve();
    JoystickButton a = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    JoystickButton y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton b = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    JoystickButton x = new JoystickButton(m_driverController, XboxController.Button.kX.value);
=======
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final PS4Controller operator = new PS4Controller(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    private final JoystickButton in = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    private final JoystickButton out = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(m_driverController,
            XboxController.Button.kLeftBumper.value);

    /* Subsystems */
<<<<<<< HEAD
    private final Shooter m_shooter = new Shooter(20, 21);
    private final Intake m_intake = new Intake();
    private final Pivot m_pivot = new Pivot();
=======
    private final Swerve s_swerve = new Swerve();
    public final Shooter m_shooter = new Shooter(20, 21);
    public final Intake m_intake = new Intake();
    public final Pivot m_pivot = new Pivot();
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_swerve,
                        () -> m_driverController.getRawAxis(translationAxis),
                        () -> m_driverController.getRawAxis(strafeAxis),
                        () -> m_driverController.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the trigger bindings
        configureBindings();
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
<<<<<<< HEAD

        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        a.whileTrue(new InstantCommand(m_intake::intake)).whileFalse(new InstantCommand(m_intake::stop));
        y.whileTrue(new InstantCommand(m_intake::Outtake)).whileFalse(new InstantCommand(m_intake::stop));

=======
        in.whileTrue(new InstantCommand(Shooter::in)).whileFalse(new InstantCommand(Shooter::stop));
        out.whileTrue(new InstantCommand(Shooter::out)).whileFalse(new InstantCommand(Shooter::stop));
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

        zeroGyro.onTrue(new InstantCommand(() -> s_swerve.zeroHeading()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
<<<<<<< HEAD
    //     // An example command will be run in autonomous
    //     // return Autos.exampleAuto(m_exampleSubsystem);
=======
    // // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901
    // }
}
