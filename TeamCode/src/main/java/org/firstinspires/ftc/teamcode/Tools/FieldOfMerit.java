/*
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FieldOfMerit {

    private static double botHeading;
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

        double X = Peripherals.getLimelightX();
        double Y = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYawDegrees();

        if (X != 0 || Y != 0) {

            currentState = "Vision";

            fieldX = X;
            fieldY = Y;
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            Drive.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }

        else {


            fieldX = Peripherals.getOtosX();
            fieldY = Peripherals.geOtostY();
            theta = Peripherals.getYawDegrees();
            botHeading = Peripherals.getYaw();
            currentState = "Mouse";
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }
    }

public double getBotHeading(){return botHeading;}
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
*//*
*/
/*
*//*

*/
/*

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
        Peripherals.resetYaw();
    }

    public static void processTags() {
        double limelightX = Peripherals.getLimelightX();
        double limelightY = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYawDegrees();

        if (isValidLimelightData(limelightX, limelightY)) {
            currentState = "Vision";
            fieldX = limelightX;
            fieldY = limelightY;
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            Peripherals.resetYaw();
            Drive.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        } else {
            currentState = "Mouse";
            Peripherals.update();
           *//*
*/
/*

*//*

*/
/* fieldX = FinalPose.x + Peripherals.getOtosX();
            fieldY = FinalPose.y + Peripherals.geOtostY();
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            FinalPose.setfinalPose(fieldX, fieldY, theta);*//*
*/
/*
*//*

*/
/*

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
*//*
*/
/*

package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

public class FieldOfMerit {

    private static double botHeading;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    public static String currentState = "Odometry Pods";

    public static void initialize(HardwareMap hardwareMap) {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);
        Peripherals.resetYaw();
    }

    public static void processTags() {
        double limelightX = Peripherals.getLimelightX();
        double limelightY = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYawDegrees();

        if (isValidLimelightData(limelightX, limelightY)) {
            currentState = "Vision";
            fieldX = limelightX;
            fieldY = limelightY;
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            Peripherals.resetYaw();
            resetMousePosition(fieldX, fieldY);
            Drive.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        } else {
            currentState = "Mouse";
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(FinalPose.x, FinalPose.y, theta);
            Peripherals.mouse.setPosition(currentPosition);
            Peripherals.update();
            fieldX = Peripherals.getOtosX();
            fieldY = Peripherals.geOtostY();
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }
    }

    private static boolean isValidLimelightData(double x, double y) {
        return x != 0 || y != 0;
    }

    private static void resetMousePosition(double x, double y) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, 0);
        Peripherals.mouse.setPosition(currentPosition);
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
*/

package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

public class FieldOfMerit {

    private static double botHeading;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    public static String currentState = "Odometry Pods";

    public static void initialize(HardwareMap hardwareMap) {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);
Peripherals.configureOtos();
        Peripherals.resetYaw();
    }

    public static void processTags() {
        double limelightX = Peripherals.getLimelightX();
        double limelightY = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYawDegrees();

        if (isValidLimelightData(limelightX, limelightY)) {
            currentState = "Vision";
            fieldY = limelightX;
            fieldX = limelightY;
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            Peripherals.resetYaw();
            Peripherals.setPosition(fieldX,fieldY,theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        } else {

            currentState = "Mouse";
            fieldX = Peripherals.getOtosX();
            fieldY = Peripherals.getOtosY();
            theta = robotYaw;
            botHeading = Peripherals.getYaw();
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }
    }

    private static boolean isValidLimelightData(double x, double y) {
        return x != 0 || y != 0;
    }

    private static void resetMousePosition(double x, double y, double yaw) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, yaw);
        Peripherals.mouse.setPosition(currentPosition);
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
/*
package org.firstinspires.ftc.teamcode.Tools;

import static org.firstinspires.ftc.teamcode.Subsystems.Peripherals.mouse;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FieldOfMerit {

    private static double botHeading;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static double lastFieldX = 0;
    private static double lastFieldY = 0;
    private static double lastTheta = 0;

    public static String currentState = "Mouse";

    public static void initialize(HardwareMap hardwareMap) {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);
        Peripherals.resetYaw();
    }

    public static void processTags() {
        double limelightX = Peripherals.getLimelightX();
        double limelightY = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYawDegrees();

       /* if (isValidLimelightData(limelightX, limelightY)) {
            currentState = "Vision";
            fieldX = limelightX;
            fieldY = limelightY;
            theta = robotYaw;
            botHeading = Peripherals.getYaw();



            Peripherals.setPosition(fieldX, fieldY, theta);
            FinalPose.setfinalPose(fieldX, fieldY, theta);
        }*//*
            currentState = "Mouse";
         fieldX = Peripherals.getOtosX();
         fieldY = Peripherals.getOtosY();
         theta = robotYaw;
         Peripherals.update();

    }

    private static boolean isValidLimelightData(double x, double y) {
        return x != 0 || y != 0;
    }

    public static double getBotHeading() {
        return botHeading;
    }

    public static double getFieldX() {
        return fieldX;
    }

    public static double getFieldY() {
        return fieldY;
    }

    public static double getTheta() {
        return theta;
    }
}*/
