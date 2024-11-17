package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class FinalPose extends Object {
    public static double x;
    public static double y;
    public static double Yaw;

    // Set the final pose with given x, y, and yaw values
    public static void setFinalPose(double X, double Y, double yaw) {
        y = X;
        x = Y;
        Yaw = yaw;
    }

    // Update pose by using the Mouse class to get the latest X, Y, and Theta
    public static void poseUpdate() {
        Mouse.update();
        x = Mouse.getX();    // Get the updated X position from Mouse class
        y = Mouse.getY();    // Get the updated Y position from Mouse class
        Yaw = Mouse.getTheta();  // Get the updated Yaw (Theta) from Mouse class
    }

    public static void Reset(){
        Mouse.configureOtos();
    }

    public static double getX() {
        return x;
    }
    public static double getY(){
        return y;
    }
    public static double getYaw(){
        return Yaw;
    }
}
