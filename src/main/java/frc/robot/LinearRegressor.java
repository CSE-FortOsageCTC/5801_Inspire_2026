package frc.robot;

import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;

public class LinearRegressor {
    private Deque<Double> queue;

    private int count = 5;

    private double xSum = (count * (1 + count))/2;
    private double ySum = 0;
    private double xxSum = 0;
    private double xySum = 0;
    private double slope = 0;
    private double intercept = 0;

    public LinearRegressor(double initialValue) {
        queue = new LinkedList<>();

        for(int i = 0; i < count; i++) {
            queue.add(initialValue);
            ySum += initialValue;
            xxSum += i * i;
            xySum += i * initialValue;
        }
    }
    public void setNextPosition(double nextValue) {
        queue.add(nextValue);
        double lastX = queue.remove();
        ySum += nextValue;
        ySum -= lastX;

        xySum = 0;
        int counter = 0;
        for (Double value : queue){
            xySum += value * counter;
        }
        slope = (count * xySum - xSum * ySum) / (count * xxSum - xSum * xSum);
        intercept = (ySum / count) - (slope * xSum) / count;
    }
    public double getNextPosition() {
        return count * slope + intercept;
    }
    public double getSlope() {
        return slope;
    }
}
