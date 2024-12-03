package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FieldOfMerit {

    private static double botHeading;
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    public static String currentState = "Mouse";

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
            fieldX = limelightX;
            fieldY = limelightY;
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
    }}