package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Mouse {
    public static double fieldX;
    public static double fieldY;
    public static double theta;
    public static SparkFunOTOS mouse;
    public static SparkFunOTOS.Pose2D field;

    // Initialize the mouse sensor
    public static void init(HardwareMap hardwareMap) {
        // Get the mouse instance from the hardware map
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        // Configure the OTOS sensor settings
        configureOtos();

    }

    // Configure the SparkFun OTOS settings
    private static void configureOtos() {
        mouse.setLinearUnit(DistanceUnit.METER);
        mouse.setAngularUnit(AngleUnit.DEGREES);
        // distance from center of robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setOffset(offset);
        mouse.calibrateImu();
        mouse.resetTracking();
        //camara
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setPosition(currentPosition);
    }

    // Update the field values by calling getPosition() only once
    public static void update() {

            // Call getPosition() once and store the result in the field object
            field = mouse.getPosition();

            // Update global X, Y, and theta (heading) positions from the sensor's field values
            fieldX = Math.round(field.x * 1000) / 1000.0;
            fieldY = Math.round(field.y * 1000) / 1000.0;
            theta = field.h;  // No need to round theta, it’s in radians

    }

    // Get the X position (already updated in the update() method)
    public static double getX() {
        return fieldX;
    }

    // Get the Y position (already updated in the update() method)
    public static double getY() {
        return fieldY;
    }

    // Get the Theta (heading) (already updated in the update() method)
    public static double getTheta() {
        return theta;
    }
}
