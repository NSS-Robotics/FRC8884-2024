package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import java.util.function.BooleanSupplier;

public class PlsWork extends Command {

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

    public PlsWork(
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
        PIDController thetaController = new PIDController(
            0.12,
            0,
            0
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("AUTO");
        s_swerve.setPose(traj[0].getInitialPose());

        Command[] swerveCommands = new Command[trajCount];
        for (int i = 0; i < trajCount; i++) {
            swerveCommands[i] =
                Choreo.choreoSwerveCommand(
                    // Choreo trajectory from above
                    traj[i],
                    // A function that returns the current field-relative pose of the robot: your
                    // wheel or vision odometry
                    s_swerve::getPose,
                    // PIDController for
                    // field-relative X
                    // translation (input: X error
                    // in meters,
                    // output: m/s).
                    new PIDController(
                        0.12,
                        0.0,
                        0.0
                    ),
                    // PIDController for
                    // field-relative Y
                    // translation (input: Y error
                    // in meters,
                    // output: m/s).
                    new PIDController(
                        0.12,
                        0.0,
                        0.0
                    ),
                    // PID constants to correct for rotation
                    // error
                    thetaController,
                    // needs to be robot-relative
                    (ChassisSpeeds speeds) ->
                        s_swerve.drive(
                            new Translation2d(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond
                            ),
                            speeds.omegaRadiansPerSecond,
                            true,
                            false
                        ),
                    // Whether or not to mirror the path based on alliance (this assumes the path is
                    // created for the blue alliance)
                    fieldmirror,
                    // The subsystem(s) to require, typically your drive subsystem only
                    s_swerve
                );
        }

        Command theTraj =
                Choreo.choreoSwerveCommand(
                    // Choreo trajectory from above
                    Choreo.getTrajectory(pathName),
                    // A function that returns the current field-relative pose of the robot: your
                    // wheel or vision odometry
                    s_swerve::getPose,
                    // PIDController for
                    // field-relative X
                    // translation (input: X error
                    // in meters,
                    // output: m/s).
                    new PIDController(
                        0.12,
                        0.0,
                        0.0
                    ),
                    // PIDController for
                    // field-relative Y
                    // translation (input: Y error
                    // in meters,
                    // output: m/s).
                    new PIDController(
                        0.12,
                        0.0,
                        0.0
                    ),
                    // PID constants to correct for rotation
                    // error
                    thetaController,
                    // needs to be robot-relative
                    (ChassisSpeeds speeds) ->
                        s_swerve.drive(
                            new Translation2d(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond
                            ),
                            speeds.omegaRadiansPerSecond,
                            true,
                            false
                        ),
                    // Whether or not to mirror the path based on alliance (this assumes the path is
                    // created for the blue alliance)
                    fieldmirror,
                    // The subsystem(s) to require, typically your drive subsystem only
                    s_swerve
                );
        return Commands.sequence(
            new InstantCommand(s_swerve::zeroGyro),
            theTraj,
            
            s_swerve.run(() ->
                s_swerve.drive(new Translation2d(0, 0), 0, true, false)
            )
        );
    }
}
