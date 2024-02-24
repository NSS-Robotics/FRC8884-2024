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

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.*;

public class EstTauto extends Command {

    private final Feeder m_feeder;
    private final Intake m_intake;
    private final Limelight l_Limelight;
    private final Pivot m_Pivot;
    private final Shooter m_shooter;
    private final Swerve s_Swerve;
    private int stopPoints;
    private ChoreoTrajectory[] traj;
    private BooleanSupplier fieldmirror;

    public EstTauto(
        String pathName,
        int stopPoints,
        Feeder m_feeder,
        Intake m_intake,
        Limelight l_Limelight,
        Pivot m_Pivot,
        Shooter m_shooter,
        Swerve s_Swerve,
        BooleanSupplier fieldmirror
        
        ) {
        traj = new ChoreoTrajectory[stopPoints];
        for (int i = 0; i < stopPoints; i++) {
            String trajName = i == 0 ? pathName : pathName + "." + i; 
            this.traj[i] = Choreo.getTrajectory(trajName + ".traj");
        } 
        
        this.stopPoints = stopPoints;
        this.m_feeder = m_feeder;
        this.m_intake = m_intake;
        this.l_Limelight = l_Limelight;
        this.m_Pivot = m_Pivot;
        this.m_shooter = m_shooter;
        this.s_Swerve = s_Swerve;
        this.fieldmirror = fieldmirror;
        addRequirements(s_Swerve);
    }

    public Command followTrajectory() {
        var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("AUTO");
        s_Swerve.setPose(traj[0].getInitialPose());

        Command[] swerveCommands = new Command[stopPoints];
        for (int i = 0; i < stopPoints; i++) {
            Command swerveCommand = Choreo.choreoSwerveCommand(
                traj[i], // Choreo trajectory from above
                s_Swerve::getPose, // A function that returns the current field-relative pose of the robot: your
                                        // wheel or vision odometry
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                            // translation (input: X error in meters,
                                                                                            // output: m/s).
                new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                            // translation (input: Y error in meters,
                                                                                            // output: m/s).
                thetaController, // PID constants to correct for rotation
                                    // error
                (ChassisSpeeds speeds) -> s_Swerve.drive( // needs to be robot-relative
                    new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                    speeds.omegaRadiansPerSecond,
                    false,
                    false
                    ),
                
                fieldmirror, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
                s_Swerve // The subsystem(s) to require, typically your drive subsystem only
            );
        }

        return Commands.sequence(
            new InstantCommand(s_Swerve::zeroHeading),
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(intake::outtake)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new InstantCommand(intake::stop)),
            // new ParallelDeadlineGroup(new WaitCommand(1.5), new MidConeNode(elevator)),
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(intake::intake)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new InstantCommand(intake::stop)),
            // new ParallelDeadlineGroup(new WaitCommand(1.5), new BottomNode(elevator)),
            //Commands.runOnce(() -> s_Swerve.setPose(traj[0].getInitialPose())),
            swerveCommands[0],
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(intake::outtake)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new InstantCommand(intake::stop)),
            //Commands.runOnce(() -> s_Swerve.setPose(traj[1].getInitialPose())),
            swerveCommands[1],
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(intake::outtake)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new InstantCommand(intake::stop)),
            // new ParallelDeadlineGroup(new WaitCommand(1.5), new MidConeNode(elevator)),
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(intake::intake)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new InstantCommand(intake::stop)),
            // new ParallelDeadlineGroup(new WaitCommand(1.5), new BottomNode(elevator)),
            s_Swerve.run(() -> s_Swerve.drive(
                new Translation2d(0, 0),
                0,
                false,
                false)
                )
        );
    }
}
