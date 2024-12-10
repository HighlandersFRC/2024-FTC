/*
package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Commands.Command;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static final double PIVOT_TICKS_PER_ROTATION = 537.6 / 5;
    public static final double PIVOT_STARTING_ANGLE = -14.85;
    public static final double PIVOT_FEED_FORWARD = 0.6;

    public static double ARM_BALANCE_OFFSET = 68;

    public static double getOffsetFromVoltage(double voltage){
        return 5.03 + -4950*voltage + -4731*Math.pow(voltage, 2) + -2098*Math.pow(voltage, 3) + -286*Math.pow(voltage, 4);
    }
    public static double yCorrected(double AY) {
        return AY - ((0.172 * AY) + 0.00307);
    }

    public static double xCorrected(double AX) {
        return AX - (0.181 * AX + -0.00049);
    }

    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();
public static class PickupData{
    public int elevatorPose;
    public int pivotPose;

    public PickupData(int elevatorPose, int pivotPose){
        this.elevatorPose = elevatorPose;
        this.pivotPose = pivotPose;
    }
}

public static final Map<Integer, PickupData> pickupMap = new HashMap<>();
static {
    pickupMap.put(959, new PickupData(959, -12));
    pickupMap.put(1370, new PickupData(1370, -10));

}
    public static class AprilTagData {
        public double positionX;
        public double positionY;
        public double size;
        public double tagangle;

        public AprilTagData(double positionX, double positionY, double size, double tagangle) {

            this.positionY = positionY;
            this.size = size;
            this.tagangle = tagangle;
        }
    }

    public static double yOffset(double x) {
        return -0.276 + 0.394 * x + (-0.114 * x * x) + (0.01 * x * x * x);
    }

    public static final Map<Integer, AprilTagData> aprilTagMap = new HashMap<>();

    static {
        aprilTagMap.put(14, new AprilTagData(3.048, 3.66, 0.1016, 0));
        aprilTagMap.put(15, new AprilTagData(3.66, 1.83, 0.1016, 0));
        aprilTagMap.put(16, new AprilTagData(3.048, 0, 0.1016, 0));
    }

    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;

}*/
package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Commands.Command;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Constants {
    public static final double PIVOT_TICKS_PER_ROTATION = 537.6 / 5;
    public static final double PIVOT_STARTING_ANGLE = -14.85;
    public static final double PIVOT_FEED_FORWARD = 0.8;
 public static final double ELEVATOR_MAX = 1500;
    public static double ARM_BALANCE_OFFSET = 68;

    public static double getOffsetFromVoltage(double voltage){
        return 5.03 + -4950*voltage + -4731*Math.pow(voltage, 2) + -2098*Math.pow(voltage, 3) + -286*Math.pow(voltage, 4);
    }
    public static double yCorrected(double AY) {
        return AY - ((0.172 * AY) + 0.00307);
    }

    public static double xCorrected(double AX) {
        return AX - (0.181 * AX + -0.00049);
    }

    public static HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
    public static HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();
    public static class PickupData{
        public int pivotPose;

        public PickupData( int pivotPose){
            this.pivotPose = pivotPose;
        }
    }

    public static final Map<Integer, PickupData> pickupMap = new HashMap<>();
    static {
        pickupMap.put(959, new PickupData( -12));
        pickupMap.put(1370, new PickupData( -10));

    }
    public static class AprilTagData {
        public double positionX;
        public double positionY;
        public double size;
        public double tagangle;

        public AprilTagData(double positionX, double positionY, double size, double tagangle) {

            this.positionY = positionY;
            this.size = size;
            this.tagangle = tagangle;
        }
    }

    public static double yOffset(double x) {
        return -0.276 + 0.394 * x + (-0.114 * x * x) + (0.01 * x * x * x);
    }

    public static final Map<Integer, AprilTagData> aprilTagMap = new HashMap<>();

    static {
        aprilTagMap.put(14, new AprilTagData(3.048, 3.66, 0.1016, 0));
        aprilTagMap.put(15, new AprilTagData(3.66, 1.83, 0.1016, 0));
        aprilTagMap.put(16, new AprilTagData(3.048, 0, 0.1016, 0));
    }

    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;

}