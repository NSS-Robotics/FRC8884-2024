package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.*;

public class BaseAuto extends Command {

    protected final Feeder m_feeder;
    protected final Intake m_intake;
    protected final Limelight l_Limelight;
    protected final Pivot m_Pivot;
    protected final Shooter m_shooter;
    protected final Swerve s_Swerve;
    protected int stopPoints;
    protected ChoreoTrajectory[] traj;
    protected BooleanSupplier fieldmirror;

    public BaseAuto(
            String pathName,
            int stopPoints,
            Feeder m_feeder,
            Intake m_intake,
            Limelight l_Limelight,
            Pivot m_pivot,
            Shooter m_shooter,
            Swerve s_swerve,
            BooleanSupplier fieldmirror) {
        traj = new ChoreoTrajectory[stopPoints];
        for (int i = 0; i < stopPoints; i++) {
            String trajName = i == 0 ? pathName : pathName + "." + i;
            this.traj[i] = Choreo.getTrajectory(trajName);
        }

        this.stopPoints = stopPoints;
        this.m_feeder = m_feeder;
        this.m_intake = m_intake;
        this.l_Limelight = l_Limelight;
        this.m_Pivot = m_pivot;
        this.m_shooter = m_shooter;
        this.s_Swerve = s_swerve;
        this.fieldmirror = fieldmirror;
        addRequirements(s_Swerve);
    }

    public Command getCommands(Command[] swerveCommands) {
        return Commands.sequence();
    }

    public Command followTrajectory() {
        PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("AUTO");
        s_Swerve.setPose(traj[0].getInitialPose());

        Command[] swerveCommands = new Command[stopPoints];
        for (int i = 0; i < stopPoints; i++) {
            swerveCommands[i] = Choreo.choreoSwerveCommand(
                    traj[i], // Choreo trajectory from above
                    s_Swerve::getPose, // A function that returns the current field-relative pose of the robot: your
                                       // wheel or vision odometry
                    new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for
                                                                                        // field-relative X
                                                                                        // translation (input: X error
                                                                                        // in meters,
                                                                                        // output: m/s).
                    new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for
                                                                                        // field-relative Y
                                                                                        // translation (input: Y error
                                                                                        // in meters,
                                                                                        // output: m/s).
                    thetaController, // PID constants to correct for rotation
                                     // error
                    (ChassisSpeeds speeds) -> s_Swerve.drive( // needs to be robot-relative
                            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                            speeds.omegaRadiansPerSecond,
                            false,
                            false),

                    fieldmirror, // Whether or not to mirror the path based on alliance (this assumes the path is
                                 // created for the blue alliance)
                    s_Swerve // The subsystem(s) to require, typically your drive subsystem only
            );
        }

        return getCommands(swerveCommands);
    }
}
