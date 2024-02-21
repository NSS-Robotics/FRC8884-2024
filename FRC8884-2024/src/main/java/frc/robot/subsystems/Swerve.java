// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

<<<<<<< HEAD
// import frc.robot.SwerveModule;
// import frc.robot.Constants;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
=======
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;

<<<<<<< HEAD
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

// public class Swerve extends SubsystemBase {
//     public SwerveDriveOdometry swerveOdometry;
//     public SwerveModule[] mSwerveMods;
//     public AHRS gyro;

//     public Swerve() {
//         gyro = new AHRS(SPI.Port.kMXP);
//         gyro.zeroYaw();

<<<<<<< HEAD
//         mSwerveMods = new SwerveModule[] {
//             new SwerveModule(0, Constants.Swerve.Mod0.constants),
//             new SwerveModule(1, Constants.Swerve.Mod1.constants),
//             new SwerveModule(2, Constants.Swerve.Mod2.constants),
//             new SwerveModule(3, Constants.Swerve.Mod3.constants)
//         };
=======
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

//         swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
//     }

<<<<<<< HEAD
//     public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
//         SwerveModuleState[] swerveModuleStates =
//             Constants.Swerve.swerveKinematics.toSwerveModuleStates(
//                 fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
//                                     translation.getX(), 
//                                     translation.getY(), 
//                                     rotation, 
//                                     getHeading()
//                                 )
//                                 : new ChassisSpeeds(
//                                     translation.getX(), 
//                                     translation.getY(), 
//                                     rotation)
//                                 );
//         SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

//         for(SwerveModule mod : mSwerveMods){
//             mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
//         }
//     }    

//     /* Used by SwerveControllerCommand in Auto */
//     public void setModuleStates(SwerveModuleState[] desiredStates) {
//         SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
//         for(SwerveModule mod : mSwerveMods){
//             mod.setDesiredState(desiredStates[mod.moduleNumber], false);
//         }
//     }

//     public SwerveModuleState[] getModuleStates(){
//         SwerveModuleState[] states = new SwerveModuleState[4];
//         for(SwerveModule mod : mSwerveMods){
//             states[mod.moduleNumber] = mod.getState();
//         }
//         return states;
//     }

//     public SwerveModulePosition[] getModulePositions(){
//         SwerveModulePosition[] positions = new SwerveModulePosition[4];
//         for(SwerveModule mod : mSwerveMods){
//             positions[mod.moduleNumber] = mod.getPosition();
//         }
//         return positions;
//     }
=======
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
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
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

//     public Pose2d getPose() {
//         return swerveOdometry.getPoseMeters();
//     }

//     public void setPose(Pose2d pose) {
//         swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
//     }

<<<<<<< HEAD
//     public Rotation2d getHeading(){
//         return getPose().getRotation();
//     }

//     public void setHeading(Rotation2d heading){
//         swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
//     }

//     public void zeroHeading(){
//         swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
//     }
=======
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901

//     public Rotation2d getGyroYaw() {
//         return Rotation2d.fromDegrees(gyro.getYaw());
//     }

<<<<<<< HEAD
//     public void resetModulesToAbsolute(){
//         for(SwerveModule mod : mSwerveMods){
//             mod.resetToAbsolute();
//         }
//     }

//     @Override
//     public void periodic(){
//         swerveOdometry.update(getGyroYaw(), getModulePositions());

//         for(SwerveModule mod : mSwerveMods){
//             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
//             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
//             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
//         }
//     }
// }
=======
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public Command exampleMethodCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'exampleMethodCommand'");
    }
}
>>>>>>> 134c3586448a895d0245ff20c5ea9ed20107d901
