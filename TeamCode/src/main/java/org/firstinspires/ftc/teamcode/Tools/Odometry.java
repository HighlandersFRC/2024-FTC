package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;

public class Odometry {
    private double leftx, rightx, x, y, theta;
    private double lastX, lastY, lastTheta;
    private double wheelDiameter = 3.77953;
    private I2cDevice leftX, RightX, Y;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }

    public void update(){

    }
}