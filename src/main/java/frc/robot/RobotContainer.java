// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.FourPieceAmp;
import frc.robot.autos.FourPieceMid;
import frc.robot.autos.ThreePieceMid;
import frc.robot.autos.TwoPiece;
import frc.robot.commands.*;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

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
        public final Pivot m_pivot = new Pivot(s_swerve, m_feeder);
        private boolean alliance = s_swerve.isRed();
        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<Integer> m_y_int_chooser = new SendableChooser<>();
        // ALLIANCE COLOUR IS RED

        private final TwoPiece twoPieceMidRed = new TwoPiece(
                        "PlxWork",
                        1,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        true);

        private final TwoPiece twoPieceMidBlue = new TwoPiece(
                        "PlxWork",
                        1,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        false);

        private final ThreePieceMid threePieceMidRed = new ThreePieceMid(
                        "ThreePieceMidPlxWork",
                        3,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        true);

        private final ThreePieceMid threePieceMidBlue = new ThreePieceMid(
                        "ThreePieceMidPlxWork",
                        3,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        false);

        private final FourPieceMid fourPieceMidRed = new FourPieceMid(
                        "FourPieceMidPlxWork",
                        4,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        true);

        private final FourPieceMid fourPieceMidBlue = new FourPieceMid(
                        "FourPieceMidPlxWork",
                        4,
                        m_feeder,
                        m_intake,
                        l_limelight_april,
                        m_pivot,
                        m_shooter,
                        s_swerve,
                        l_candle,
                        () -> false,
                        false);

        private final FourPieceAmp fourPieceAmpPlxWork = new FourPieceAmp(
                        "FourPieceAmpPlxWork",
                        6,
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

                m_chooser.addOption(
                                "TwoPieceMidRed",
                                twoPieceMidRed.followTrajectory());
                m_chooser.addOption("TwoPieceMidBlue", twoPieceMidBlue.followTrajectory());
                m_chooser.addOption(
                                "ThreePieceMidRed",
                                threePieceMidRed.followTrajectory());
                m_chooser.addOption(
                                "ThreePieceMidBlue",
                                threePieceMidBlue.followTrajectory());
                // m_chooser.addOption("ThreePieceAmpPlxWork",
                // threePieceAmpSidePlxWork.followTrajectory());
                // m_chooser.addOption("ThreePieceSourcePlxWork",
                // threePieceSourceSidePlxWork.followTrajectory());
                m_chooser.addOption(
                                "FourPieceMidRed",
                                fourPieceMidRed.followTrajectory());
                m_chooser.addOption(
                                "FourPieceMidBlue",
                                fourPieceMidBlue.followTrajectory());
                m_chooser.addOption(
                                "FourPieceAmpPlxWork",
                                fourPieceAmpPlxWork.followTrajectory());

                // m_chooser.setDefaultOption("TestAuto", testAuto.followTrajectory());
                // m_y_int_chooser.addOption("85", 85);
                // m_y_int_chooser.addOption("86", 86);
                // m_y_int_chooser.addOption("87", 87);
                // m_y_int_chooser.setDefaultOption("88", 88);
                // m_y_int_chooser.addOption("89", 89);
                // m_y_int_chooser.addOption("90", 90);
                // m_y_int_chooser.addOption("91", 91);
                // m_y_int_chooser.addOption("92", 92);
                // m_y_int_chooser.addOption("93", 93);

                SmartDashboard.putData("Auto mode", m_chooser);
                // SmartDashboard.putData("Y-Int", m_y_int_chooser);

        }

        public void setYInt() {
                m_pivot.setYInt(m_y_int_chooser.getSelected());
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
                rTrigger.whileTrue(
                                new SpeakerShoot(m_shooter, m_pivot, m_feeder, l_candle));
                rTrigger.toggleOnFalse(new InstantCommand(() -> l_candle.ledsOff()));
                x.whileTrue(new NoteOuttake(m_intake, m_feeder, l_candle));
                lTrigger.whileTrue(
                                new SequentialCommandGroup(
                                                new NoteIntake(m_intake, m_feeder, l_candle)));
                rb.whileTrue(new AmpShoot(m_shooter, m_pivot, m_feeder, l_candle));
                up.whileTrue(new ClimbPos(m_pivot));
                down.whileTrue(new IntakePos(m_pivot));

                /* Operator Buttons */
                triangle.toggleOnTrue(
                                new InstantCommand(() -> l_candle.toggle(255, 40, 0)));
                circle.toggleOnTrue(
                                new InstantCommand(() -> l_candle.toggle(255, 192, 203)));
                square.whileTrue(new NoteAlign(s_swerve, l_limelight_intake));
                cross.onTrue(new InstantCommand(() -> l_candle.reset()));
                r1.whileTrue(
                                new Lob(m_pivot, m_shooter, m_feeder, Constants.PivotConstants.UpLobRotations,
                                                l_candle));
                l1.whileTrue(
                                new Lob(
                                                m_pivot,
                                                m_shooter,
                                                m_feeder,
                                                Constants.PivotConstants.DownLobRotations,
                                                l_candle));
                upDawg.onFalse(new InstantCommand(() -> m_pivot.changeYInt(0.003))); // todo
                downDawg.onFalse(new InstantCommand(() -> m_pivot.changeYInt(-0.003))); // todo
                leftDawg.onFalse(new InstantCommand(() -> m_pivot.changeAmp(0.005)));
                rightDawg.onFalse(new InstantCommand(() -> m_pivot.changeAmp(-0.005)));
                r2.whileTrue(
                                new Lob(
                                                m_pivot,
                                                m_shooter,
                                                m_feeder,
                                                Constants.PivotConstants.AgainstSpeakerRotations,
                                                l_candle));
                l2.whileTrue(new SpinShooter(m_shooter));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                l_candle.rainbow();
                System.out.println("AUTO");
                alliance = s_swerve.isRed();
                System.out.println("s_swerve.isRed() = " + alliance);
                SmartDashboard.putBoolean("s_swerve.isRed()", alliance);

                return m_chooser.getSelected();
        }
}
