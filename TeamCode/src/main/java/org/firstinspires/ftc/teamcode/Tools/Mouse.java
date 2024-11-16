package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mouse {
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;


    public static void init(HardwareMap hardwareMap) {

        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");


        configureOtos();

    }


    public static void configureOtos() {
        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.0889, 0.01905, -90);
        mouse.setOffset(offset);
        mouse.setLinearScalar(1.069338135974786);
        mouse.setAngularScalar(0.989932511851);
        mouse.calibrateImu();
        mouse.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setPosition(currentPosition);
    }


    public static void update() {

        field = mouse.getPosition();

        FinalPose.x = field.x;
        FinalPose.y =field.y;
        FinalPose.Yaw = field.h;

    }


    public static double getX() {
        return fieldX;
    }


    public static double getY() {
        return fieldY;
    }


    public static double getTheta() {
        return theta;
    }
}