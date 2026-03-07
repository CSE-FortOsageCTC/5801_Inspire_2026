package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class TurretState {
    
    public double turretDegrees;
    public double hoodDegrees;
    public double initialVelocity;
    public Pose2d fieldRelativeFieldPose;

    public TurretState(double turretDegrees, double hoodDegrees, double initialVelocity, Pose2d fieldRelativeFieldPose){
        this.turretDegrees = turretDegrees;
        this.hoodDegrees = hoodDegrees;
        this.initialVelocity = initialVelocity;
        this.fieldRelativeFieldPose = fieldRelativeFieldPose;
    }

}
