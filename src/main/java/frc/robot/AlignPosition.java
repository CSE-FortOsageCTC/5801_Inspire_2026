package frc.robot;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Swerve;

public enum AlignPosition {
    LeftOffset(),
    CenterOffset(),
    RightOffset(),
    L1Offset(),
    NoPos();

    private static AlignPosition alignPosition;
    private static Pose2d alignOffset;
    private static int rotationDegrees;
    private static double rotationRadians;
    private static double correctedX;
    private static double correctedY;
    private static double tagX;
    private static double tagY;
    private static double distance;
    private static double theta;
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // private static LimeLightSubsystem s_LimeLightSubsystem =
    // LimeLightSubsystem.getRightInstance();
    private static Swerve s_Swerve = Swerve.getInstance();

    private static boolean scoring = true;

    public static AlignPosition getPosition() {

        if (alignPosition == null) {
            alignPosition = LeftOffset;
        }

        if (alignOffset == null) {
            setPosition(NoPos);
        }

        return alignPosition;
    }

    public static Pose2d getAlignOffset() {
        // SmartDashboard.putString("April Align Offset", "(" + alignOffset.getX() + ", " + alignOffset.getY() + ")");
        return alignOffset;
    }

    public static void setPosition(AlignPosition alignPos) {
        // SmartDashboard.putString("Align Pos", alignPos.toString());
        alignPosition = alignPos;

        // int tagID = s_LimeLightSubsystem.getAprilValue();

        s_Swerve.resetAlignApril();

        // if (tagID == -1) {
        // alignOffset = s_Swerve.getEstimatedPosition();
        // return;
        // }

        // tagID = s_LimeLightSubsystem.getAprilValue();
        boolean isRed = DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get());

        System.out.println(isRed);

        rotationDegrees = (int) Math.round(getClimbingPos().getRotation().getDegrees());
        rotationRadians = (getClimbingPos().getRotation().getRadians()) - (Math.PI / 2);
        // double thetaRadians = Math.atan2(Constants.scoringDx, Constants.scoringDy);
        tagX = getClimbingPos().getX();
        tagY = getClimbingPos().getY();
        // SmartDashboard.putNumber("Tag X", tagX);
        // SmartDashboard.putNumber("Tag Y", tagY);
        // SmartDashboard.putNumber("Tag Rotation", rotationDegrees);

        distance = Math.sqrt(((Constants.climbingDx * Constants.climbingDx)) + (Constants.climbingDy * Constants.climbingDy));

        switch (alignPosition) {
            case LeftOffset:
                theta = Math.atan2(Constants.climbingDy, Constants.climbingDx) + rotationRadians;
                tagX = getClimbingPos().getX();
                tagY = getClimbingPos().getY();
                correctedPos();
                break;
            case RightOffset:
                theta = Math.atan2(Constants.climbingDy, -Constants.climbingDx) + rotationRadians;
                tagX = getClimbingPos().getX();
                tagY = getClimbingPos().getY();
                correctedPos();
                break;
            case NoPos:
                alignOffset = s_Swerve.getEstimatedPosition();
                break;
        }

        alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));

        // SmartDashboard.putNumber("Translated April X", alignOffset.getX());
        // SmartDashboard.putNumber("Translated April Y", alignOffset.getY());

    }

    public static void correctedPos() {
        correctedX = tagX + Math.cos(theta) * distance;
        correctedY = tagY + Math.sin(theta) * distance;
    }

    public static Pose2d getClimbingPos() {
        boolean isRed = DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get());
        Pose2d closestPose;
        if (isRed) {
            Pose2d tagPose = fieldLayout.getTagPose(15).get().toPose2d();
            closestPose = tagPose;
        } else {
            Pose2d tagPose = fieldLayout.getTagPose(31).get().toPose2d();
            closestPose = tagPose;
        }
        return closestPose;
    }

    public static boolean getIsScoring() {
        return scoring;
    }

    public static void setIsScoring(boolean isScoring) {
        scoring = isScoring;
    }

    AlignPosition() {
    }
}
