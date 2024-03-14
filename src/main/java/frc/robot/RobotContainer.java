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
        Trigger dpadUp = driverController.povUp();
        Trigger dpadDn = driverController.povDown();

        Trigger rb = driverController.rightBumper();
        Trigger lb = driverController.leftBumper();

        Trigger rTrigger = driverController.rightTrigger();
        Trigger lTrigger = driverController.leftTrigger();

        Trigger up = driverController.povUp();
        Trigger down = driverController.povDown();

        /* Operator Buttons */
        Trigger circle = operatorController.circle();
        Trigger square = operatorController.square();
        Trigger cross = operatorController.cross();
        Trigger triangle = operatorController.triangle();

        Trigger r1 = operatorController.R1();
        Trigger l1 = operatorController.L1();

        Trigger r2 = operatorController.R2();
        Trigger l2 = operatorController.L2();

        Trigger upDawg = operatorController.povUp();
        Trigger downDawg = operatorController.povDown();
        Trigger rightDawg = operatorController.povRight();
        Trigger leftDawg = operatorController.povLeft();

        
        /* Subsystems */
        public final Candle l_candle = new Candle();
        public final Shooter m_shooter = new Shooter();
        public final Intake m_intake = new Intake(m_shooter);
        public final Feeder m_feeder = new Feeder(m_shooter, l_candle);
        public final Limelight l_limelight_april = new Limelight("april");
        public final Limelight l_limelight_intake = new Limelight("intake");
        public final Swerve s_swerve = new Swerve(l_limelight_april);
        public final Pivot m_pivot = new Pivot(s_swerve);
        private boolean alliance = s_swerve.isRed();
        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<Integer> m_y_int_chooser = new SendableChooser<>();

        private final TwoPiece twoPiecePlxWork = new TwoPiece(
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

        private final ThreePiece threePieceMidPlxWork= new ThreePiece(
                        "PlxWork",
                        2,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false);

                        
        private final FourPiece fourPieceMidPlxWork= new FourPiece(
                        "FourPieceMidPlxWork",
                        3,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false);        

        public RobotContainer() {
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

                m_chooser.addOption("TwoPiecePlxWork", twoPiecePlxWork.followTrajectory());
                m_chooser.addOption("ThreePiecePlxWork", threePieceMidPlxWork.followTrajectory());
                m_chooser.addOption("FourPieceMidPlxWork", fourPieceMidPlxWork.followTrajectory());

                // m_chooser.setDefaultOption("TestAuto", testAuto.followTrajectory());
                m_y_int_chooser.setDefaultOption("88", 88);
                m_y_int_chooser.addOption("87", 87);
                m_y_int_chooser.addOption("89", 89);
                SmartDashboard.putData("Auto mode", m_chooser);
                SmartDashboard.putData("Y-Int", m_y_int_chooser);
                
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

        public void setYInt() {
                m_pivot.setYInt(m_y_int_chooser.getSelected());
        }

        private void configureBindings() {
                /* Driver Buttons */
                y.whileTrue(new InstantCommand(s_swerve::zeroGyro));
                rTrigger.whileTrue(new AimLimelight(s_swerve, l_limelight_april));
                rTrigger.whileTrue(new SpeakerShoot(m_shooter, m_pivot, l_candle));
                x.whileTrue(new NoteOuttake(m_intake, m_feeder, l_candle));
                lTrigger.whileTrue(new NoteIntake(m_intake, m_feeder, l_candle));
                rb.whileTrue(new AmpShoot(m_shooter, m_pivot, l_candle));
                dpadUp.whileTrue(new ClimbPos(m_pivot));
                dpadDn.whileTrue(new IntakePos(m_pivot));

                /* Operator Buttons */
                circle.whileTrue(new InstantCommand(() -> l_candle.strobe(255,255,0)));
                circle.whileFalse(new InstantCommand(() -> l_candle.ledsOff()));
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
