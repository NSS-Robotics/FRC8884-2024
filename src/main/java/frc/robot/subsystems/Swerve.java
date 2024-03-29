// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import java.util.Optional;

public class Swerve extends SubsystemBase {

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private AHRS gyro;

    private Pose2d m_pose;
    private int driveInvert;
    private Limelight l_limelight;
    private boolean limelightStatus;

    public Swerve(Limelight limelight) {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants),
        };

        swerveOdometry = createOdometry(new Pose2d(0, 0, new Rotation2d()));
        m_pose = swerveOdometry.update(getGyroYaw(), getModulePositions());
        // driveInvert = (isRed() ? 1 : -1);
        l_limelight = limelight;
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getGyroYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates,
                Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(
                    swerveModuleStates[mod.moduleNumber],
                    isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates,
                Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void zeroGyro() {
        // gyro.zeroYaw();
        gyro.reset();
        setHeading(new Rotation2d(Units.degreesToRadians(isRed() ? 180 : 0)));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return isRed()
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(360 - gyro.getYaw());
    }

    public double getGyroDegrees() {
        return gyro.getYaw();
    }

    public Pose2d getLimelightBotPose() {
        return m_pose;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public double[] getSpeakerDistances() {
        Pose2d pose = getLimelightBotPose();

        double x = (isRed() ? Constants.redSpeakerX : Constants.blueSpeakerX) -
                pose.getX();
        double y = Constants.speakerY - pose.getY();
        return new double[] { x, y };
    }

    public void turnStates(double angularSpeed) {
        var swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0,
                        0,
                        angularSpeed,
                        gyro.getRotation2d()));
        setModuleStates(swerveModuleStates);
    }

    public SwerveDriveOdometry createOdometry(Pose2d pose) {
        return new SwerveDriveOdometry(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                pose);
    }

    public void printPosData() {
        System.out.println("----------- Pose Data -----------");
        System.out.println("Pos X: " + m_pose.getX());
        System.out.println("Pos Y: " + m_pose.getY());
        System.out.println("Pos R: " + m_pose.getRotation().getDegrees());
        System.out.println("Yaw: " + gyro.getAngle());

        System.out.println("isRed: " + isRed());
    }

    public void setLimelightStatus(boolean x) {
        limelightStatus = x;
    }

    @Override
    public void periodic() {
        // if (l_limelight.tv > 0) {
        // swerveOdometry = createOdometry(l_limelight.botPose);
        // m_pose = l_limelight.botPose;
        // } else {
        // m_pose = swerveOdometry.update(getGyroYaw(), getModulePositions());
        // }

        if (l_limelight.tv > 0 && limelightStatus) {
            swerveOdometry = createOdometry(l_limelight.botPose);
            m_pose = l_limelight.botPose;
        } else {
            m_pose = swerveOdometry.update(getGyroYaw(), getModulePositions());
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " CANcoder",
                    mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Angle",
                    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("Pos X", m_pose.getX());
        SmartDashboard.putNumber("Pos Y", m_pose.getY());
        SmartDashboard.putNumber("Pos R", m_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getAngle());
    }
}
