package org.firstinspires.ftc.teamcode.PathingTool;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.json.JSONArray;
import org.json.JSONObject;

public class PathEngine {

    private JSONObject pathData;
    private static JSONArray samplePoints;
    private static double positionX;
    private static double positionY;
    private static double heading;
    private static double prevLeft;
    private static double prevRight; // Previous encoder positions for 2 motors
    private static final double TICKS_PER_METER = 300; // Example value, adjust as needed

    private HardwareMap hardwareMap;
    private CommandScheduler commandScheduler;

    private static int currentIndex;

    private static PID headingPID; // PID controller for heading

    private static final double Kp_heading = 0.1; // Proportional gain for heading PID
    private static final double Ki_heading = 0.0; // Integral gain for heading PID
    private static final double Kd_heading = 0.05; // Derivative gain for heading PID

    public PathEngine(String pathJson, HardwareMap hardwareMap) {
        try {
            this.pathData = new JSONObject(pathJson);
            this.samplePoints = pathData.getJSONArray("sampled_points");
            this.hardwareMap = hardwareMap;
            this.commandScheduler = CommandScheduler.getInstance();
            DriveSubsystem.start(hardwareMap);
            Peripherals.init(hardwareMap); // Initialize peripherals including IMU
            resetOdometry();
            this.currentIndex = 0;

            // Initialize heading PID controller
            headingPID = new PID(Kp_heading, Ki_heading, Kd_heading);
            headingPID.clamp(1); // Limit PID output to motor power range

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void resetOdometry() {
        this.positionX = 0;
        this.positionY = 0;
        this.heading = 0;
        this.prevLeft = DriveSubsystem.getLeftEncoder(); // Adjust to your actual method
        this.prevRight = DriveSubsystem.getRightEncoder(); // Adjust to your actual method
        Peripherals.resetYaw(); // Reset yaw to ensure correct heading at start
    }

    public static void update() {
        if (currentIndex < samplePoints.length()) {
            try {
                JSONObject point = samplePoints.getJSONObject(currentIndex);
                double targetX = point.getDouble("x");
                double targetY = point.getDouble("y");
                double targetHeading = point.getDouble("angle");
                double distance = calculateDistance(positionX, positionY, targetX, targetY);
                double speed = point.getDouble("x_velocity"); // Assuming speed is provided

                // Calculate required heading adjustment using PID
                double headingError = normalizeAngle(targetHeading - heading);
                double headingCorrection = headingPID.getResult(headingError);

                // Calculate motor powers with heading correction
                double[] motorPowers = calculateMotorPowers(targetX, targetY, speed, headingCorrection);
                DriveSubsystem.Drive(motorPowers[0], motorPowers[1]); // Use DriveSubsystem to move the robot

                if (isCloseEnough(positionX, positionY, targetX, targetY)) {
                    currentIndex++;
                    positionX = targetX;
                    positionY = targetY;
                    heading = targetHeading;
                }

                updateOdometry();
                telemetry(); // Print telemetry data
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private static double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    private static double[] calculateMotorPowers(double targetX, double targetY, double speed, double headingCorrection) {
        // Calculate desired velocities (kinematic model)
        double desiredVelocityLeft = speed - headingCorrection * 0.1; // Adjust coefficients as needed
        double desiredVelocityRight = speed + headingCorrection * 0.1;

        // Convert velocities to motor powers
        double maxVelocity = Math.max(Math.abs(desiredVelocityLeft), Math.abs(desiredVelocityRight));
        double leftPower = desiredVelocityLeft / maxVelocity;
        double rightPower = desiredVelocityRight / maxVelocity;

        // Limit motor powers to achievable range
        leftPower = limitPower(leftPower);
        rightPower = limitPower(rightPower);

        return new double[]{leftPower, rightPower};
    }

    private static double normalizeAngle(double angle) {
        return (angle + 180) % 360 - 180;
    }

    private static double limitPower(double power) {
        return Math.max(-1, Math.min(1, power));
    }

    private static boolean isCloseEnough(double x1, double y1, double x2, double y2) {
        return calculateDistance(x1, y1, x2, y2) < 0.1; // Adjust threshold as needed
    }

    private static void updateOdometry() {
        double leftPos = DriveSubsystem.getLeftEncoder(); // Adjust to your actual method
        double rightPos = DriveSubsystem.getRightEncoder(); // Adjust to your actual method

        double deltaLeft = leftPos - prevLeft;
        double deltaRight = rightPos - prevRight;

        double deltaForward = (deltaLeft + deltaRight) / 2.0 / TICKS_PER_METER;
        double deltaSideways = 0.0; // Assuming tank drive does not have sideways movement

        heading = Peripherals.getYaw(); // Update heading from IMU

        positionX += deltaForward * Math.cos(Math.toRadians(heading)) - deltaSideways * Math.sin(Math.toRadians(heading));
        positionY += deltaForward * Math.sin(Math.toRadians(heading)) + deltaSideways * Math.cos(Math.toRadians(heading));

        prevLeft = leftPos;
        prevRight = rightPos;
    }

    private static void telemetry() {
        // Calculate projected encoder ticks to reach next sample point
        if (currentIndex < samplePoints.length()) {
            try {
                JSONObject point = samplePoints.getJSONObject(currentIndex);
                double targetX = point.getDouble("x");
                double targetY = point.getDouble("y");

                double distanceToTarget = calculateDistance(positionX, positionY, targetX, targetY);
                int projectedTicks = (int) (distanceToTarget * TICKS_PER_METER);

                // Print telemetry
                System.out.println("Projected encoder ticks to next point: " + projectedTicks);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
