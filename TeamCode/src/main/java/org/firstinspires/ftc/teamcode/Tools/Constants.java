package org.firstinspires.ftc.teamcode.Tools;


import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.TestCommand;
import org.firstinspires.ftc.teamcode.Commands.TestCommand2;


import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


public class Constants {
    public static final double PID_X_P = 0.1;
    public static final double PID_X_I = 1;
    public static final double PID_X_D = 1;
    public static final double PID_Y_P = 0.1;
    public static final double PID_Y_I = 1;
    public static final double PID_Y_D = 1;
    public static final double PID_THETA_P = 0.1;
    public static final double PID_THETA_I = 1;
    public static final double PID_THETA_D = 1;
    public static double absoluteArmZero =0.306;
    public static double armOffset;
    public static double ArmUpPosition = 4000;
    public static double ElevatorsUpPosition = 2000;
    public static double ElevatorsDownPosition = 200;
    public static double ArmDownPosition = 200;
    public static Object SetPoints;
    public static final PID pivotPID = new PID(0.003, 0, 0);

    public static double nextX;
    public static double nextY;
    public static double nextTheta;


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


    static {
        commandMap.put("TestCommand", () -> new TestCommand());
        commandMap.put("TestCommand2", () -> new TestCommand2());
    }




    public static class AprilTagData {
        public double positionX;
        public double positionY;
        public double size;
        public double tagangle;


        public AprilTagData(double positionX, double positionY, double size, double tagangle) {
            this.positionX = positionX;
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


    public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 10;
    public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 10;
    public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;


}
