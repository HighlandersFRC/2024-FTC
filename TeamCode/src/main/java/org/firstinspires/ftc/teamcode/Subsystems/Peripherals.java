package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;


public class Peripherals extends Subsystem {

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private double xPosition = 0, yPosition = 0, theta = 0;
    private int lastLeftPosition = 0, lastRightPosition = 0;
    private double wheelDiameter = 0.1; // Wheel diameter in meters
    private double wheelBase = 0.3; // Distance between the two wheels in meters
    static IMU imu;
    static IMU.Parameters myIMUparameters;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;
    private static double fieldX;
    private static double fieldY;
    private static double Theta;

    public Peripherals(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        // Set up the IMU parameters
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,      // Z-axis angle
                                0,       // Y-axis angle
                                -90,     // X-axis angle
                                0        // acquisitionTime, not used
                        )
                )
        );

        // Initialize the IMU with the configured parameters
        imu.initialize(myIMUparameters);
    }

    public static double getYawDegrees(){
        return Theta;
    }

    public static double getYaw() {
       return Math.toRadians(Theta);
    }
    //You need to update to get values
    public static void update() {


        field = mouse.getPosition();


        fieldX = field.x;
        fieldY =field.y;
        Theta = field.h;
    }

    public static double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public static double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public static void resetYaw() {
        imu.resetYaw();
    }
}
