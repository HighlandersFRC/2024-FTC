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

        double X = Peripherals.getLimelightX();
        double Y = Peripherals.getLimelightY();
        double robotYaw = Peripherals.getYaw();

        if (X != 0 || Y != 0) {

            currentState = "Vision";

            fieldX = X;
            fieldY = Y;
            theta = robotYaw;

            Drive.setPosition(fieldX, fieldY, theta);
            FinalPose.setFinalPose(fieldX, fieldY, theta);
        }

        else {


            fieldX = Drive.getOdometryX();
            fieldY = Drive.getOdometryY();
            theta = Drive.getOdometryTheta();

            currentState = "Odometry Pods";
            FinalPose.setFinalPose(fieldX, fieldY, theta);
        }
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
