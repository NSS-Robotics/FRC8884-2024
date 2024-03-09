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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakePos;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class ExampleAuto extends Command {

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

    public ExampleAuto(
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
        for (int i = 0; i < trajCount; i++) {
            String trajName = pathName + "." + (i + 1);
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
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        PIDController thetaController = new PIDController(
            0.12,
            0,
            0
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("AUTO");
        ChoreoTrajectory traj = Choreo.getTrajectory("PlsWork");
        
        s_swerve.setPose(traj.getInitialPose());
        fieldmirror = () -> {
                        
                            return alliance.isPresent() && alliance.get() == Alliance.Red;
                    };

        
        
        
        Command theCMD = Choreo.choreoSwerveCommand(
            traj, // 
            s_swerve::getPose, // 
            new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
            new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
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

        return Commands.sequence(
            new InstantCommand(s_swerve::zeroGyro),
            Commands.runOnce(() -> s_swerve.setPose(traj.getInitialPose())),
            
            theCMD,
            new ParallelCommandGroup(new SpeakerShoot(m_shooter, m_pivot, l_candle), new WaitCommand(1)),
            new ParallelCommandGroup(new SpeakerShoot(m_shooter, m_pivot, l_candle), new NoteIntake(m_intake, m_feeder, l_candle), new WaitCommand(2)),
            new ParallelCommandGroup(new IntakePos(m_pivot)),
            new ParallelCommandGroup(new InstantCommand(m_intake::stop)),
            s_swerve.run(() -> s_swerve.drive(
                new Translation2d(0, 0),
                0,
                true,
                false
            ))
        );
                
    }
}
