package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VelocityEstimator {
    private LinearRegressor xLinearRegressor;
    private LinearRegressor yLinearRegressor;

    public VelocityEstimator(Pose2d initialPose) {
        xLinearRegressor = new LinearRegressor(initialPose.getX());
        yLinearRegressor = new LinearRegressor(initialPose.getY());
    }
    public void setNextPosition(Pose2d nextPosition) {
        xLinearRegressor.setNextPosition(nextPosition.getX());
        yLinearRegressor.setNextPosition(nextPosition.getY());
    }
    public ChassisSpeeds getVelocity() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        chassisSpeeds.vxMetersPerSecond = xLinearRegressor.getSlope() / 0.02;
        chassisSpeeds.vyMetersPerSecond = yLinearRegressor.getSlope() / 0.02;
        return chassisSpeeds;
    }
    public Pose2d getNextPosition() {
        return new Pose2d(xLinearRegressor.getNextPosition(), yLinearRegressor.getNextPosition(), new Rotation2d());
    }
}
