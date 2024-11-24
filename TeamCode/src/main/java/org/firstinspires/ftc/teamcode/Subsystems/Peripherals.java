package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

/*
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

public class Peripherals extends Subsystem {

    private static double fieldX;
    private static double fieldY;
    private static double Theta;
    public static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private double xPosition = 0, yPosition = 0, theta = 0;
    private int lastLeftPosition = 0, lastRightPosition = 0;
    private double wheelDiameter = 0.1;
    private double wheelBase = 0.3;
    static IMU imu;
    private static Limelight3A limelight;

    public Peripherals(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        configureOtos();

        imu = hardwareMap.get(IMU.class, "imu");
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.getStatus();
        limelight.start();
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


        fieldX = field.x;
        fieldY =field.y;
        Theta = field.h;

    }




    public static double getYawDegrees(){
        return Theta;
    }

    public static double getYaw() {
        return Math.toRadians(Theta);
    }
    public static double getOtosX() {
        return fieldX;
    }


    public static double geOtostY() {
        return fieldY;
    }

    public static double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public static double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public static void resetYaw() {
        mouse.resetTracking();
    }

    public static LLResult getLimelightResult() {
        return limelight.getLatestResult();
    }

    public static void updateRobotOrientation(double yaw) {
        limelight.updateRobotOrientation(yaw);
    }

    public static void stopLimelight() {
        limelight.stop();
    }

    public static double getLimelightX() {
        LLResult result = getLimelightResult();
        if (result != null && result.isValid()) {
            return result.getBotpose().getPosition().x + 1.83;
        }
        return 0;
    }

    public static double getLimelightY() {
        LLResult result = getLimelightResult();
        if (result != null && result.isValid()) {
            return result.getBotpose().getPosition().y + 1.83;
        }
        return 0;
    }



}*/
public class Peripherals extends Subsystem {

    private static double fieldX;
    private static double fieldY;
    private static double Theta;
    private static double fX;
    private static double fY;
    private static double fTheta;
    public static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private double xPosition = 0, yPosition = 0, theta = 0;
    private int lastLeftPosition = 0, lastRightPosition = 0;
    private double wheelDiameter = 0.1;
    private double wheelBase = 0.3;
    static IMU imu;
    private static Limelight3A limelight;

    public Peripherals(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        configureOtos();

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.getStatus();
        limelight.start();
    }

    public static void configureOtos() {
        if (mouse != null) {
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
    }
    public static void IDK (){
        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.0889, 0.01905, -90);
        mouse.setOffset(offset);
        mouse.setLinearScalar(1.069338135974786);
        mouse.setAngularScalar(0.989932511851);

    }

    public static void update() {

            field = mouse.getPosition();
            fieldX = field.x;
            fieldY = field.y;
            Theta = field.h;

    }

    public static double getYawDegrees() {
        return Theta;
    }
    public static void setPosition(double x, double y, double theta){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, theta);
        mouse.setPosition(currentPosition);
    }
    public static double getYaw() {
        return Math.toRadians(fTheta);
    }

    public static double getOtosX() {
        return fieldX;
    }

    public static double getOtosY() {
        return fieldY;
    }

    public static double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public static double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public static void resetYaw() {
        if (mouse != null) {
            mouse.resetTracking();
        }
    }

    public static LLResult getLimelightResult() {
        return limelight.getLatestResult();
    }

    public static void updateRobotOrientation(double yaw) {
        limelight.updateRobotOrientation(yaw);
    }

    public static void stopLimelight() {
        limelight.stop();
    }

    public static double getLimelightX() {
        LLResult result = getLimelightResult();
        if (result != null && result.isValid()) {
            return result.getBotpose().getPosition().x + 1.83;
        }
        return 0;
    }

    public static double getLimelightY() {
        LLResult result = getLimelightResult();
        if (result != null && result.isValid()) {
            return result.getBotpose().getPosition().y + 1.83;
        }
        return 0;
    }


}
