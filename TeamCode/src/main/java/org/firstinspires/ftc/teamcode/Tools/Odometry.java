package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class Odometry {
    private static double x;
    private static double y;
    private double theta;
    private double lastLeftEncoder, lastRightEncoder, lastYEncoder;
    private double wheelDiameter = 3.77953; // inches
    private double ticksPerRevolution = 1440; // Adjust based on your encoder
    private double ticksPerMeter = (ticksPerRevolution) / (Math.PI * wheelDiameter * 0.0254); // Hardcoded ticks per meter value
    private DcMotor lateralLeft, lateralRight, Y;

    public void initialize(HardwareMap hardwareMap){
        lateralLeft = hardwareMap.get(DcMotor.class, "lateralLeft");
        lateralRight = hardwareMap.get(DcMotor.class, "lateralRight");
        Y = hardwareMap.get(DcMotor.class, "Y");

        lastLeftEncoder = lateralLeft.getCurrentPosition();
        lastRightEncoder = lateralRight.getCurrentPosition();
        lastYEncoder = Y.getCurrentPosition();
    }

    public void setStartPosition(double startX, double startY, double startTheta) {
        x = startX;
        y = startY;
        theta = startTheta;
    }

    public void update() {
        double leftEncoder = lateralLeft.getCurrentPosition();
        double rightEncoder = lateralRight.getCurrentPosition();
        double yEncoder = Y.getCurrentPosition();

        double deltaLeft = (leftEncoder - lastLeftEncoder) / ticksPerMeter;
        double deltaRight = (rightEncoder - lastRightEncoder) / ticksPerMeter;
        double deltaY = (yEncoder - lastYEncoder) / ticksPerMeter;

        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
        lastYEncoder = yEncoder;

        double deltaTheta = (deltaLeft - deltaRight) / wheelDiameter;
        theta += deltaTheta;

        double deltaX = deltaY * Math.sin(theta);
        double deltaYPosition = deltaY * Math.cos(theta);

        x += deltaX;
        y += deltaYPosition;
    }

    public static double getX() {
        return x;
    }

    public static double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
