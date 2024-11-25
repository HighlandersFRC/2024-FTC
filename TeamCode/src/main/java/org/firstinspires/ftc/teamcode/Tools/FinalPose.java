package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FinalPose extends Object{
    public static double x;
    public static double y;
    public static double Yaw;
     public static double botHeading;
    public static void setfinalPose(double X, double Y, double yaw){
        x = X;
        y = Y;
        Yaw = yaw;
        botHeading = botHeading;
    }
    public static void poseUpdate() {
        FieldOfMerit.processTags();
        Drive.update();
       Mouse.update();
    }

}
