// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String limelightBack = "limelight-back";
    public static final String limelightFront = "limelight-front";
    public static final String limelightSky = "limelight-sky";
    
    public static final double feedForwardAngle = 30;

    public static final double maximumVoltage = 10;

    public static final double climbingDx = Units.inchesToMeters(16.125); //TODO: adjust values as needed
    public static final double climbingDy = Units.inchesToMeters(39); //TODO: adjust values as needed

    public static final double autoRotateRange = 60;

    // position (x and y) of the hub from a top down perspective. height (z) of target of the hub
    public static final Pose3d redHubPosition = new Pose3d(11.901424, 4.034536, 1.8288, new Rotation3d());
    public static final Pose3d blueHubPosition = new Pose3d(4.625594, 4.034536, 1.8288, new Rotation3d());
    public static final Pose3d redNorthShuttleTarget = new Pose3d(14.207236, 6.031992, 0, new Rotation3d());
    public static final Pose3d redSouthShuttleTarget = new Pose3d(14.207236, 2.010664, 0, new Rotation3d());
    public static final Pose3d blueNorthShuttleTarget = new Pose3d(2.305812, 6.031992, 0, new Rotation3d());
    public static final Pose3d blueSouthShuttleTarget = new Pose3d(2.305812, 2.010664, 0, new Rotation3d());
    public static final double redAllianceLineX = 3.977894;
    public static final double blueAllianceLineX = 12.563094;
    public static final double redTrenchAreaLeftX = 3.977894;
    public static final double redTrenchAreaRightX = 5.222494;
    public static final double TrenchAreaTopY = 6.79069;
    public static final double TrenchAreaBottomY = 1.278636;
    public static final double blueTrenchAreaLeftX = 10.318494;
    public static final double blueTrenchAreaRightX = 12.563094;
    public static final double minimumHubDist = 0.61; // in meters
    public static final double maximumHubDist = 5.4; // in meters
    public static final double minimumHoodAngle = 50; // in degrees
    public static final double maximumHoodAngle = 90; // in degrees
    public static final double minimumHoodShotDegrees = 70;
    public static final double maximumHoodShotDegrees = 80;
    public static final double totalHoodRangeDegrees = 40;
    public static final double minimumHoodEncoder = 0;
    public static final double maximumHoodEncoder = 70; // TODO: Figure this out!
    public static final double hoodEncoderPerDegree = maximumHoodEncoder / totalHoodRangeDegrees; // TODO: Figure this out too!
    public static final double maximumBallSpeed = 7.7; // (in m/s) TODO: Test m/s of ball on release at max flywheel speed
    public static final double minimumSwivelEncoder = 0;
    public static final double maximumSwivelEncoder = 175; // TODO: Find this out :)
    public static final double swivelEncoderToAbsolute = 49.2854;
    public static final double totalSwivelRangeDegrees = 255; // In Degrees TODO: Find this out :)
    public static final double swivelEncoderPerDegrees = 0.67367; // maximumSwivelEncoder / totalSwivelRangeDegrees; // TODO: Find this out :)
    public static final Pose2d turretPoseRobotReletive = new Pose2d(0.2032, -0.1016, Rotation2d.fromDegrees(90)); // TODO: find x and y meters offset that the robot is from the center of the robot
    public static final double stickDeadband = 0.125;
    public static final double flywheelMaxVelocity = -83.5;


    //Set Points
    public static double primaryL1ClimbSetpoint = 0; //TODO: find this
    public static double secondaryL1ClimbSetpoint = 0; //TODO: find this

    public static final int numberOfLEDs = 8; //COULD CHANGE


    

  public static class OperatorConstants {
    
}
public static final int kDriverControllerPort = 0;
public static final class Swerve {
        public static final int pigeonID = 10;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                // COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(30); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(24); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

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
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

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

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1; // TODO: This must be tuned to specific robot || og value 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */

        public static final double maxSpeed = 11; // TODO: This must be tuned to specific robot


        /** Radians per Second */
        public static final double maxAngularVelocity = 13.0; // TODO: This must be tuned to specific robot
        public static final double panicRotation = 25.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;


                    // max Distance from robot - wrist angle shenanigans
        public static final double XLimit = 38 - 10;

        // public static final double
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(127.529296875); 
                                                                                              
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 14;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(167.080078125); 
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 16;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.296875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(132.275390625);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 32;
        public static final double kPYController = 32;
        public static final double kPThetaController = 8;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
  }
