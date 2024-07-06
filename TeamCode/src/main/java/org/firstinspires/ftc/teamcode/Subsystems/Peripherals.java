package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Peripherals extends Subsystem {

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private double xPosition = 0, yPosition = 0, theta = 0;
    private int lastLeftPosition = 0, lastRightPosition = 0;
    private double wheelDiameter = 0.1; // Wheel diameter in meters
    private double wheelBase = 0.3; // Distance between the two wheels in meters
    private static IMU imu;

    public Peripherals(String name) {
        super(name);
    }


    public static void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void updatePosition() {
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        int leftChange = leftPosition - lastLeftPosition;
        int rightChange = rightPosition - lastRightPosition;

        double leftDistance = leftChange * (Math.PI * wheelDiameter) / 1440.0; // Assuming 1440 ticks per revolution
        double rightDistance = rightChange * (Math.PI * wheelDiameter) / 1440.0;

        double distance = (leftDistance + rightDistance) / 2.0;
        double deltaTheta = (rightDistance - leftDistance) / wheelBase;

        theta += deltaTheta;
        xPosition += distance * Math.cos(Math.toRadians(theta));
        yPosition += distance * Math.sin(Math.toRadians(theta));

        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;

    }

    public double getXPosition() {
        updatePosition();
        return xPosition;
    }

    public double getYPosition() {
        updatePosition();
        return yPosition;
    }

    public static double getYawDegrees(){
        return  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public static double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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
