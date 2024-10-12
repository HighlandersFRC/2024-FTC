

package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Mouse {
    private static Mouse instance;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    // Private constructor to prevent instantiation
    private Mouse() { }

    // Singleton method to get the only instance of the Mouse class
    public static Mouse getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Mouse();
            init(hardwareMap);
            configureOtos();
        }
        return instance;
    }

    // Initialize the mouse sensor
    public static void init(HardwareMap hardwareMap) {
        // Get the mouse instance from the hardware map
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        // Configure the OTOS sensor settings
        configureOtos();
    }

    // Configure the SparkFun OTOS settings
    public static void configureOtos() {
        mouse.setLinearUnit(DistanceUnit.INCH);
        mouse.setAngularUnit(AngleUnit.DEGREES);
        // Distance from center of robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setOffset(offset);
        mouse.calibrateImu();
        mouse.resetTracking();
        // Camera
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
