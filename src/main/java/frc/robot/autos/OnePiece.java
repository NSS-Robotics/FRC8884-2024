package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;


public class OnePiece extends Command {

        protected final Feeder m_feeder;
        protected final Intake m_intake;
        protected final Limelight l_limelight_april;
        protected final Pivot m_pivot;
        protected final Shooter m_shooter;
        protected final Swerve s_swerve;
        protected final Candle l_candle;

        protected int trajCount;
        protected ChoreoTrajectory[] traj;
        protected BooleanSupplier fieldmirror;

        public OnePiece(
                        Feeder m_feeder,
                        Intake m_intake,
                        Limelight l_limelight_april,
                        Pivot m_pivot,
                        Shooter m_shooter,
                        Swerve s_swerve,
                        Candle l_candle,
                        BooleanSupplier fieldmirror) {
                
                this.m_feeder = m_feeder;
                this.m_intake = m_intake;
                this.l_limelight_april = l_limelight_april;
                this.m_pivot = m_pivot;
                this.m_shooter = m_shooter;
                this.s_swerve = s_swerve;
                this.fieldmirror = fieldmirror;
                this.l_candle = l_candle;
                addRequirements(
                                m_feeder,
                                m_intake,
                                l_limelight_april,
                                m_pivot,
                                m_shooter,
                                s_swerve);
        }

    public Command followTrajectory() {
        PIDController thetaController = new PIDController(
                0.12,
                0,
                0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.sequence(
                new InstantCommand(s_swerve::zeroGyro),
                
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(new WaitCommand(2), new Lob(m_pivot, m_shooter, 51)),
                        new ParallelDeadlineGroup(new WaitCommand(1), new Lob(m_pivot, m_shooter, 51),
                                new NoteIntake(m_intake, m_feeder, l_candle))),
                new WaitCommand(1)
        );
    }
}