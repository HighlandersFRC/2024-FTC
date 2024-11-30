/*
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FieldOfMerit {


    private static double fieldY;
    private static double fieldX;
    private static double theta;
    public static String currentState = "Odometry Pods";



    public static void initialize(HardwareMap hardwareMap) {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);

        Peripherals.resetYaw();
    }



    public static void processTags() {

*/
/*        double X = Peripherals.getLimelightX();
        double Y = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYaw();*//*

       */
/* double X = 0;
        double Y = 0;
        double robotYaw = 0;

        if (X != 0 || Y != 0) {

            currentState = "Vision";

            fieldX = X;
            fieldY = Y;
            theta = robotYaw;

            Drive.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }

        else {


            fieldX = Drive.getOdometryX();
            fieldY = Drive.getOdometryY();
            theta = Drive.getOdometryTheta();

            currentState = "Odometry Pods";
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }*//*

    }

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }

    public double getTheta() {
        return theta;
    }


}






*/

package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FieldOfMerit {

    private static double botHeading;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    public static String currentState = "Odometry Pods";

    public static void initialize(HardwareMap hardwareMap) {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();
        Peripherals.resetYaw();
    }

    public static void processTags() {
        double limelightX = Peripherals.getLimelightX();
        double limelightY = Peripherals.getLimelightY();
        double robotYaw = Mouse.getTheta();

        if (isValidLimelightData(limelightX, limelightY)) {
            currentState = "Vision";
            fieldY = limelightX;
            fieldX = limelightY;
            theta = robotYaw;
            Peripherals.resetYaw();
            Mouse.setPosition(fieldX,fieldY,theta);
            Mouse.update();
            FinalPose.setfinalPose(fieldX, fieldY, theta);

        } else {

            currentState = "Mouse";
            fieldX = Mouse.getX();
            fieldY = Mouse.getY();
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            Mouse.update();
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }
    }

    private static boolean isValidLimelightData(double x, double y) {
        return x != 0 || y != 0;
    }

    public double getBotHeading() {
        return botHeading;
    }

    public double getFieldX() {
        return fieldX;
    }

    public double getFieldY() {
        return fieldY;
    }

    public double getTheta() {
        return theta;
    }
}