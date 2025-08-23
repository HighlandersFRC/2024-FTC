package org.firstinspires.ftc.teamcode.Tools;

public class Odo {
    private double x, y, theta; // Robot's position and orientation in inches and degrees
    private double previousLeft, previousRight, previousBack; // Previous encoder values
    private static final double TICKS_TO_INCHES = 15.3; // Conversion factor

    // Constructor
    public Odo() {
        this.x = 0;
        this.y = 0;
        this.theta = 0;
        this.previousLeft = 0;
        this.previousRight = 0;
        this.previousBack = 0;
    }

    // Method to update the robot's position
    public void updateOdometry(double leftLateral, double rightLateral, double back) {
        double leftDelta = (leftLateral - previousLeft) * TICKS_TO_INCHES;
        double rightDelta = (rightLateral - previousRight) * TICKS_TO_INCHES;
        double backDelta = (back - previousBack) * TICKS_TO_INCHES;

        // Calculate change in orientation (in degrees)
        double deltaTheta = (rightDelta - leftDelta) / 2; // Assuming equal distance between wheels
        theta += deltaTheta;

        // Normalize theta between -180 and 180 degrees
        theta = normalizeTheta(theta);

        // Calculate average distance traveled by lateral pods
        double avgDistance = (leftDelta + rightDelta) / 2;

        // Update position based on the average distance and current orientation
        x += avgDistance * Math.cos(Math.toRadians(theta));
        y += avgDistance * Math.sin(Math.toRadians(theta));

        // Adjust for rear pod movement
        x += backDelta * Math.cos(Math.toRadians(theta + 90));
        y += backDelta * Math.sin(Math.toRadians(theta + 90));

        // Update previous values
        previousLeft = leftLateral;
        previousRight = rightLateral;
        previousBack = back;

        // Print current position and orientation
        System.out.printf("X: %.2f, Y: %.2f, Theta: %.2f%n", x, y, theta);
    }

    // Method to normalize angle between -180 and 180
    private double normalizeTheta(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // Getters for position and orientation
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
