package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/** Aims at the target using the limelight TX value (turns tx deg) */
public class AimLimelight extends PIDCommand {

    private Swerve swerve;
    private Limelight limelight;

    public AimLimelight(Swerve swerve, Limelight limelight) {
        super(
                new PIDController(0.1, 0, 0.01),
                () -> getAngle(swerve),
                0.0,
                tx -> swerve.turnStates(tx),
                swerve);
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(this.swerve, this.limelight);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(2.5);

        System.out.println("Align With Limelight - Start");
    }

    public static double getAngle(Swerve swerve) {
        double[] dists = swerve.getSpeakerDistances();
        double angleToSpeaker = Math.toDegrees(Math.atan(dists[1] / dists[0]));
        double rotationZ = swerve
                .getLimelightBotPose()
                .getRotation()
                .getDegrees();

        if (!swerve.isRed()) {
            angleToSpeaker += 180;
            if (rotationZ < 0) {
                rotationZ += 360;
            }
        }
        System.out.println(
                "\n-----------------------------speaker align data:");
        System.out.println("Speaker angle final:    " + angleToSpeaker);
        System.out.println("Rotation angle initial: " + rotationZ);

        return angleToSpeaker - rotationZ;
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);
        System.out.println("Align With Limelight - End");
        super.end(interrupted);
    }
}
