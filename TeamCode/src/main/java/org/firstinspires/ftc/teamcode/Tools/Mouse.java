package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mouse {
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    // Initialize the mouse sensor
    public static void init(HardwareMap hardwareMap) {
        // Get the mouse instance from the hardware map
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        // Configure the OTOS sensor settings
        configureOtos();

    }

    // Configure the SparkFun OTOS settings
    private static void configureOtos() {
        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
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
        if (mouse != null) {
            // Call getPosition() once and store the result in the field object
            field = mouse.getPosition();

            // Update global X, Y, and theta (heading) positions from the sensor's field values
            fieldX = Math.round(field.x * 1000) / 1000.0;
            fieldY = Math.round(field.y * 1000) / 1000.0;
            theta = field.h;  // No need to round theta, it’s in radians
        } else {
            throw new IllegalStateException("Mouse sensor is not initialized.");
        }
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