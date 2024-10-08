// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double stickDeadband = 0.1;

    public static final double blueSpeakerX = Units.inchesToMeters(0) + 0.17;
    public static final double redSpeakerX = Units.inchesToMeters(652.73) - 0.17;
    public static final double speakerY = Units.inchesToMeters(218.42);

    public static class ControllerConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class GlobalVariables {

        public static final int outputRangeMax = 1;
        public static final int outputRangeMin = -1;
    }

    public static class Swerve {

        public static final boolean invertGyro = true;

        /** (6.75 : 1) */
        public static final double L2 = (6.75 / 1.0);

        public static final COTSTalonFXSwerveConstants chosenModuleDRIVE = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(
                        COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
        public static final COTSTalonFXSwerveConstants chosenModuleTURN = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(
                COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        public static final double stickDeadband = 0.1;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26);
        public static final double wheelBase = Units.inchesToMeters(26);
        public static final double wheelCircumference = chosenModuleDRIVE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModuleDRIVE.driveGearRatio;
        public static final double angleGearRatio = chosenModuleTURN.angleGearRatio;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {

            public static final int driveMotorID = 3;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                    -148.5); // was -148.5
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {

            public static final int driveMotorID = 6;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                    109.423828125); // was -111.55
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {

            public static final int driveMotorID = 12;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                    162.25); // was 162.25
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {

            public static final int driveMotorID = 9;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                    221.046875); // was -141.5
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset);
        }
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class IntakeConstants {

        public static final int inner = 16;
        public static final int outer = 17;
        public static final int currentLimit = 0;
        public static final double holdingVoltage = 0;
        public static final double kS = 0.00000000045983;
        public static final double kV = 0.000022016;
        public static final double kA = 0.0000025834;
        public static final double kP = 3.22626e-05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double FF = 1.5e-3;
        public static final double speed = 350;
    }

    public static final class FeederConstants {

        public static final int feederOut = 19;
        public static final int feederIn = 18;
        public static final int currentLimit = 0;
        public static final double holdingVoltage = 1;
        public static final double kS = 0.43816;
        public static final double kV = 0.0010703;
        public static final double kA = 0.00012;
        public static final double kP = 3.3219E-05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double FF = 2.5e-3;
        public static final double speed = 200;
        public static final double feedSpeed = 600;
    }

    public static final class PivotConstants {

        public static final int pivotMotor = 14;
        public static final int followerMotor = 15;
        public static final int currentLimit = 0;
        public static final int MaxRotation = 2;
        public static final double kS = 0; // TODO
        public static final double kV = 0; // TODO
        public static final double kP = 60;
        public static final double kI = 0.00;
        public static final double kD = 0.1;
        public static final double climbP = 30;
        public static final double climbI = 0.00;
        public static final double climbD = 0;
        public static final double PivotIntakeRotation = 0;
        public static final double PivotDownRotation = 0;
        public static final double PivotAgainstRotations = 0;
        public static final double TrapAgainstRotations = 0;
        public static final double AmpRotations = 1.5;
        public static final double DownLobRotations = 0.2;
        public static final double ClimbRotations = 1.1;
        public static final double UpLobRotations = 0.55;
        public static final double AgainstSpeakerRotations = 0.65;
    }

    public static final class ShooterConstants {

        public static final int shooterMotor = 20;
        public static final int followerMotor = 21;
        public static final int currentLimit = 0;
        public static final double holdingVoltage = 0;
        public static final double kS = 0.3237;
        public static final double kV = 0.1171;
        public static final double kA = 0.023513;
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double FF = 1.5e-3;
        public static final double speed = 5200; // Should be 3500 for trap
        public static final double revSpeed = 2000;
        public static final double ampspeed = 1600;
    }

    public static final class CandleConstants {

        public static final int candleLeft = 22;
        public static final int candleRight = 23;
    }

    public static final class LaserCanConstants {

        public static final int laserCan = 24;
    }
}
