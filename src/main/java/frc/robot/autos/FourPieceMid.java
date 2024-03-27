package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AimLimelight;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.PivotDown;
import frc.robot.commands.SpeakerShootForAuto;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class FourPieceMid extends Command {

    protected final String pathName;
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

    public FourPieceMid(
        String pathName,
        int trajCount,
        Feeder m_feeder,
        Intake m_intake,
        Limelight l_limelight_april,
        Pivot m_pivot,
        Shooter m_shooter,
        Swerve s_swerve,
        Candle l_candle,
        BooleanSupplier fieldmirror
    ) {
        traj = new ChoreoTrajectory[trajCount];
        String fileName = pathName + (s_swerve.isRed() ? "RED" : "BLUE");
        for (int i = 0; i < trajCount; i++) {
            String trajName = fileName + "." + (i + 1);
            this.traj[i] = Choreo.getTrajectory(trajName);
            System.out.println(trajName);
        }
        this.pathName = pathName;
        this.trajCount = trajCount;
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
            s_swerve
        );
    }

    public Command followTrajectory() {
        PIDController thetaController = new PIDController(0.12, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("AUTO");

        Command[] theCMDs = new Command[trajCount];
        for (int i = 0; i < trajCount; i++) {
            s_swerve.setPose(traj[i].getInitialPose());
            theCMDs[i] =
                Choreo.choreoSwerveCommand(
                    traj[i], //
                    s_swerve::getPose, //
                    new PIDController(
                        Constants.AutoConstants.kPXController,
                        0.0,
                        0.0
                    ), //
                    new PIDController(
                        Constants.AutoConstants.kPXController,
                        0.0,
                        0.0
                    ), //
                    thetaController, //
                    (ChassisSpeeds speeds) -> //
                        s_swerve.drive(
                            new Translation2d(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond
                            ),
                            speeds.omegaRadiansPerSecond,
                            true,
                            false
                        ),
                    () -> false, //
                    s_swerve //
                );
        }


        return Commands.sequence(
            new InstantCommand(s_swerve::zeroGyro),
            new InstantCommand(() -> m_feeder.setShouldRev(true)),
            new InstantCommand(() -> s_swerve.setLimelightStatus(true)),

            // Shoot up against speaker
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), 
                new SpeakerShootForAuto(m_shooter, m_pivot, m_feeder, l_candle), 
                new AimLimelight(s_swerve, l_limelight_april),
                new SequentialCommandGroup(
                    new WaitCommand(1.5), 
                    new ParallelDeadlineGroup(
                        new WaitCommand(1), new NoteIntake(m_intake, m_feeder, l_candle)
                    )
                )
            ),
            new WaitCommand(1),

            // traj 1 pick up amp side note
            // new InstantCommand(() -> s_swerve.setLimelightStatus(false)),
            new InstantCommand(() -> s_swerve.setPose(traj[0].getInitialPose())),
            new ParallelDeadlineGroup(theCMDs[0], new PivotDown(m_pivot), new NoteIntake(m_intake, m_feeder, l_candle), new WaitCommand(2)),
            new InstantCommand(() -> s_swerve.drive(
                    new Translation2d(0, 0),
                    0,
                    true,
                    false)),
            // new InstantCommand(() -> s_swerve.setLimelightStatus(true)),

            // Shoot
            new ParallelDeadlineGroup(new WaitCommand(1.5), new NoteIntake(m_intake, m_feeder, l_candle)),
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), 
                new SpeakerShootForAuto(m_shooter, m_pivot, m_feeder, l_candle), 
                new AimLimelight(s_swerve, l_limelight_april),
                new SequentialCommandGroup(
                    new WaitCommand(1.5), 
                    new ParallelDeadlineGroup(
                        new WaitCommand(1), new NoteIntake(m_intake, m_feeder, l_candle)
                    )
                )
            ),
                    

            // traj 2 pick up mid note
            // new InstantCommand(() -> s_swerve.setLimelightStatus(true)),
            new InstantCommand(() -> s_swerve.setPose(traj[1].getInitialPose())),
            new ParallelDeadlineGroup(theCMDs[1], new PivotDown(m_pivot), new NoteIntake(m_intake, m_feeder, l_candle), new WaitCommand(2)),
            new InstantCommand(() -> s_swerve.drive(
                    new Translation2d(0, 0),
                    0,
                    true,
                    false)),
            // new InstantCommand(() -> s_swerve.setLimelightStatus(true)),

            // Shoot
            new ParallelDeadlineGroup(new WaitCommand(1.5), new NoteIntake(m_intake, m_feeder, l_candle)),
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), 
                new SpeakerShootForAuto(m_shooter, m_pivot, m_feeder, l_candle),
                new AimLimelight(s_swerve, l_limelight_april),
                new SequentialCommandGroup(
                    new WaitCommand(1.5), 
                    new ParallelDeadlineGroup(
                        new WaitCommand(1), new NoteIntake(m_intake, m_feeder, l_candle)
                    )
                )
            ),

            // traj 3 pick up stage side note
            // new InstantCommand(() -> s_swerve.setLimelightStatus(true)),
            new InstantCommand(() -> s_swerve.setPose(traj[2].getInitialPose())),
            new ParallelDeadlineGroup(theCMDs[2], new PivotDown(m_pivot), new NoteIntake(m_intake, m_feeder, l_candle), new WaitCommand(2)),
            new InstantCommand(() -> s_swerve.drive(
                    new Translation2d(0, 0),
                    0,
                    true,
                    false)),
            // new InstantCommand(() -> s_swerve.setLimelightStatus(true)),

            
            // Shoot
            new ParallelDeadlineGroup(new WaitCommand(1.5), new NoteIntake(m_intake, m_feeder, l_candle)),
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), 
                new SpeakerShootForAuto(m_shooter, m_pivot, m_feeder, l_candle), 
                new AimLimelight(s_swerve, l_limelight_april),
                new SequentialCommandGroup(
                    new WaitCommand(1.5), 
                    new ParallelDeadlineGroup(
                        new WaitCommand(1), new NoteIntake(m_intake, m_feeder, l_candle)
                    )
                )
            ),
            new PivotDown(m_pivot)
        );
    }
}
