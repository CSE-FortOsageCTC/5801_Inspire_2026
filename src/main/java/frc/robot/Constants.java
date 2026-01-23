// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String limelightRight = "limelight-right";
    public static final String limelightLeft = "limelight-left";
    public static final String limelightSky = "limelight-sky";

    // position (x and y) of the hub from a top down perspective. height (z) of target of the hub
    public static final Pose3d hubPosition = new Pose3d(4.625594, 4.034536, 1.8288, null);
    public static final double minimumHubDist = 0.61; // in meters
    public static final double maximumHubDist = 5.4; // in meters
    public static final double minimumHoodAngle = 75; // in degrees
    public static final double maximumHoodAngle = 85; // in degrees
    public static final double maximumBallSpeed = 10.7; // (in m/s) TODO: Test m/s of ball on release at max flywheel speed
    public static final double minimumSwivelEncoder = 0;
    public static final double maximumSwivelEncoder = 180; // TODO: Find this out :)
    public static final double swivelEncoderPerDegrees = 0; // TODO: Find this out :)
    public static final double totalSwivelRange = 180; // In Degrees TODO: Find this out :)
    public static final Pose2d turretPoseRobotReletive = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // TODO: find x and y meters offset that the robot is from the center of the robot

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
