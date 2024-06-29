package org.firstinspires.ftc.teamcode.PathingTool;

import java.util.List;

public class PathData {

    public static class SampledPoint {
        public double time;
        public double x;
        public double y;
        public double angle;
        public double x_velocity;
        public double y_velocity;
        public double angular_velocity;

        public SampledPoint(double time, double x, double y, double angle, double x_velocity, double y_velocity, double angular_velocity) {
            this.time = time;
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.x_velocity = x_velocity;
            this.y_velocity = y_velocity;
            this.angular_velocity = angular_velocity;
        }
    }
}
