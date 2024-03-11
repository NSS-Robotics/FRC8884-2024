// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.OperatingSystemMXBean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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

        private final CommandXboxController driverController = new CommandXboxController(
                        Constants.ControllerConstants.kDriverControllerPort);

        private final CommandPS4Controller operatorController = new CommandPS4Controller(
                        Constants.ControllerConstants.kOperatorControllerPort);
        // private final Swerve m_exampleSubsystem = new Swerve();
        // Replace with CommandPS4Controller or CommandJoystick if needed

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kRightY.value;
        private final int strafeAxis = XboxController.Axis.kRightX.value;
        private final int rotationAxis = XboxController.Axis.kLeftX.value;

        /* Driver Buttons */
        Trigger y = driverController.y();
        Trigger x = driverController.x();
        Trigger a = driverController.a();
        Trigger b = driverController.b();

        Trigger rb = driverController.rightBumper();
        Trigger lb = driverController.leftBumper();

        Trigger rTrigger = driverController.rightTrigger();
        Trigger lTrigger = driverController.leftTrigger();

        /* Operator Buttons */
        Trigger circle = operatorController.circle();
        Trigger square = operatorController.square();
        Trigger cross = operatorController.cross();
        Trigger triangle = operatorController.triangle();

        Trigger r1 = operatorController.R1();
        Trigger l1 = operatorController.L1();

        Trigger r2 = operatorController.R2();
        Trigger l2 = operatorController.L2();

        Trigger up = operatorController.povUp();
        Trigger down = operatorController.povDown();
        /* Subsystems */
        public final Candle l_candle = new Candle();
        public final Shooter m_shooter = new Shooter();
        public final Intake m_intake = new Intake(m_shooter);
        public final Feeder m_feeder = new Feeder(m_shooter);
        public final Limelight l_limelight_april = new Limelight("april");
        public final Limelight l_limelight_intake = new Limelight("intake");
        public final Swerve s_swerve = new Swerve(l_limelight_april);
        public final Pivot m_pivot = new Pivot(s_swerve);
        private boolean alliance = s_swerve.isRed();
        private final SendableChooser<Command> m_chooser = new SendableChooser<>();

        

        private final ExampleAuto plsWork = new ExampleAuto(
                        "PlxWork",
                        1,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false);

        public RobotContainer() {
                l_candle.setLEDs(170, 247, 250);
                s_swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_swerve,
                                                () -> driverController.getRawAxis(translationAxis),
                                                () -> driverController.getRawAxis(strafeAxis),
                                                () -> -driverController.getRawAxis(rotationAxis) * 0.75,
                                                () -> false));
                l_limelight_april.setPipeline(0);

                // Configure the trigger bindings
                configureBindings();

                m_chooser.addOption("PlsWork", plsWork.followTrajectory());
                //m_chooser.setDefaultOption("TestAuto", testAuto.followTrajectory());

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
                y.whileTrue(new InstantCommand(s_swerve::zeroGyro));
                rTrigger.whileTrue(new AimLimelight(s_swerve, l_limelight_april));
                rTrigger.whileTrue(new SpeakerShoot(m_shooter, m_pivot, l_candle));
                x.whileTrue(new NoteOuttake(m_intake, m_feeder, l_candle));
                lTrigger.whileTrue(new NoteIntake(m_intake, m_feeder, l_candle));
                rb.whileTrue(new AmpShoot(m_shooter, m_pivot, l_candle));
                
                /* Operator Buttons */
                up.whileTrue(new ClimbPos(m_pivot));
                down.whileTrue(new IntakePos(m_pivot));
                

                // rb.whileTrue(new Feed(m_shooter, m_feeder));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *  
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                System.out.println("AUTO");
                alliance = s_swerve.isRed();
                System.out.println("s_swerve.isRed() = " + alliance);
                SmartDashboard.putBoolean("s_swerve.isRed()", alliance);

                return m_chooser.getSelected();
        }
}
