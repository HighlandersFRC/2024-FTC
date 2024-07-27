package org.firstinspires.ftc.teamcode.Tools;

public class PoseMerging {
    public double[] odometryPosition(){
        return new double[]{Odometry.getOdometryX(), Odometry.getOdometryY(), Odometry.getOdometryTheta()};
    }
    public double[] cameraPosition(){
        return new double[]{Odometry.getOdometryX(), Odometry.getOdometryY(), Odometry.getOdometryTheta()};
    }
    public double[] opticalPosition(){
        return new double[]{Odometry.getOdometryX(), Odometry.getOdometryY(), Odometry.getOdometryTheta()};
    }
}
