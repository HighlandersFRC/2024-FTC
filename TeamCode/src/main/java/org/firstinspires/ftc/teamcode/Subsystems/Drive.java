package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Drive extends Subsystem{
    static DcMotor frontLeftMotor;
    static DcMotor backLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backRightMotor;

    public static final double TICKS_PER_REV = 2000;
    public static final double WHEEL_DIAMETER = 48 / 1000.0;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double TRACK_WIDTH = 12.5 * 25.4 / 1000.0;
    public static final double CORRECTION_FACTOR = 1;

    public static double x = 0.0;
    public static double y = 0.0;
    public static double theta = 0.0;

    public static int lastLeftPos = 0;
    public static int lastRightPos = 0;
    public static int lastCenterPos = 0;

    public static PID xPID = new PID(1, 0, 0);
    public static PID yPID = new PID(1, 0, 0);
    public static PID thetaPID = new PID(1, 0, 0);

    public static void initialize(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        backLeftMotor = hardwareMap.dcMotor.get("left_back");
        frontRightMotor = hardwareMap.dcMotor.get("right_front");
        backRightMotor = hardwareMap.dcMotor.get("right_back");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
    }

    public static void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);

        System.out.println("Drive Powers: FL=" + leftFrontPower + ", FR=" + rightFrontPower +
                ", BL=" + leftBackPower + ", BR=" + rightBackPower);
    }
    public static void autoDrive(Vector vector, double turnRadiansPerSec) {
        update();

        double heading = Math.toRadians(Peripherals.getYawDegrees());

        // Adjust vector based on the current heading
        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);
        double xAdjusted = vector.getI() * cosHeading - vector.getJ() * sinHeading;
        double yAdjusted = vector.getI() * sinHeading + vector.getJ() * cosHeading;

        // Calculate the power for each wheel
        double frontLeftPower = yAdjusted + xAdjusted + turnRadiansPerSec;
        double frontRightPower = yAdjusted - xAdjusted - turnRadiansPerSec;
        double backLeftPower = yAdjusted - xAdjusted + turnRadiansPerSec;
        double backRightPower = yAdjusted + xAdjusted - turnRadiansPerSec;

        // Normalize the powers if necessary
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }


        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // Optional: Print motor powers for debugging
        RobotLog.d("autoDrive Powers: FL=" + frontLeftPower + ", FR=" + frontRightPower +
                ", BL=" + backLeftPower + ", BR=" + backRightPower);
    }
    public static void vectorDrive(Vector vector, double turnRadiansPerSec) {
        update();

        double heading = Math.toRadians(Peripherals.getYawDegrees());

        double xAdjusted = vector.getI() * Math.cos(heading) - vector.getJ() * Math.sin(heading);
        double yAdjusted = vector.getI() * Math.sin(heading) + vector.getJ() * Math.cos(heading);

        double frontLeftPower = yAdjusted + xAdjusted + turnRadiansPerSec;
        double frontRightPower = yAdjusted - xAdjusted - turnRadiansPerSec;
        double backLeftPower = yAdjusted - xAdjusted + turnRadiansPerSec;
        double backRightPower = yAdjusted + xAdjusted - turnRadiansPerSec;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        RobotLog.d("vectorDrive Powers: FL=" + frontLeftPower + ", FR=" + frontRightPower +
                ", BL=" + backLeftPower + ", BR=" + backRightPower);
    }


    public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
                                          JSONArray pathPoints) throws JSONException {
        JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
        int targetIndex = pathPoints.length() - 1;
        for (int i = currentIndex; i < pathPoints.length(); i++) {
            JSONObject point = pathPoints.getJSONObject(i);
            double velocityMag = Math
                    .sqrt(Constants.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS
                            * (Math.pow(point.getDouble("x_velocity"), 2) + Math.pow(point.getDouble("y_velocity"), 2))
                            + Constants.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS
                            * Math.pow(point.getDouble("angular_velocity"), 2));
            if (!insideRadius(currentX - point.getDouble("x"), currentY - point.getDouble("y"),
                    currentTheta - point.getDouble("angle"),
                    Constants.AUTONOMOUS_LOOKAHEAD_DISTANCE * velocityMag + 0.01)) {
                targetIndex = i;
                targetPoint = pathPoints.getJSONObject(i);
                break;
            }
        }
        double targetX = targetPoint.getDouble("x"), targetY = targetPoint.getDouble("y"),
                targetTheta = targetPoint.getDouble("angle");

        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);

        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        double xVelNoFF = xPID.getResult();
        double yVelNoFF = yPID.getResult();
        double thetaVelNoFF = -thetaPID.getResult();

        double feedForwardX = targetPoint.getDouble("x_velocity");
        double feedForwardY = targetPoint.getDouble("y_velocity");
        double feedForwardTheta = -targetPoint.getDouble("angular_velocity")/3;

        Number[] velocityArray = new Number[] {
                feedForwardX + xVelNoFF,
                -(feedForwardY + yVelNoFF),
                feedForwardTheta + thetaVelNoFF,
                targetIndex,
        };

        double velocityMag = Math
                .sqrt(Math.pow(targetPoint.getDouble("x_velocity"), 2) + Math.pow(targetPoint.getDouble("y_velocity"), 2));
        return velocityArray;
    }

    private boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
    }
    public static void setCurrentPosition(double x, double y, double theta) {
        x = x;
        y = y;
        theta = theta;
    }

    public static void update() {
        int currentLeftPos = getLeftEncoder();
        int currentRightPos = getRightEncoder();
        int currentCenterPos = getCenterEncoder();

        // Handle encoder overflow
        if (Math.abs(currentLeftPos - lastLeftPos) > TICKS_PER_REV / 2) {
            if (currentLeftPos > lastLeftPos) {
                lastLeftPos += TICKS_PER_REV;
            } else {
                lastLeftPos -= TICKS_PER_REV;
            }
        }
        if (Math.abs(currentRightPos - lastRightPos) > TICKS_PER_REV / 2) {
            if (currentRightPos > lastRightPos) {
                lastRightPos += TICKS_PER_REV;
            } else {
                lastRightPos -= TICKS_PER_REV;
            }
        }
        if (Math.abs(currentCenterPos - lastCenterPos) > TICKS_PER_REV / 2) {
            if (currentCenterPos > lastCenterPos) {
                lastCenterPos += TICKS_PER_REV;
            } else {
                lastCenterPos -= TICKS_PER_REV;
            }
        }

        // Calculate encoder deltas
        int deltaLeft = currentLeftPos - lastLeftPos;
        int deltaRight = currentRightPos - lastRightPos;
        int deltaCenter = currentCenterPos - lastCenterPos;

        // Update last encoder positions
        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastCenterPos = currentCenterPos;

        // Calculate distances
        double distanceLeft = (deltaLeft / (double) TICKS_PER_REV) * WHEEL_CIRCUMFERENCE * CORRECTION_FACTOR;
        double distanceRight = (deltaRight / (double) TICKS_PER_REV) * WHEEL_CIRCUMFERENCE * CORRECTION_FACTOR;
        double distanceCenter = (deltaCenter / (double) TICKS_PER_REV) * WHEEL_CIRCUMFERENCE * CORRECTION_FACTOR;

        // Sanity check for distance
        if (Math.abs(distanceLeft) > WHEEL_CIRCUMFERENCE || Math.abs(distanceRight) > WHEEL_CIRCUMFERENCE || Math.abs(distanceCenter) > WHEEL_CIRCUMFERENCE) {
            RobotLog.e("Invalid encoder readings: L=" + distanceLeft + ", R=" + distanceRight + ", C=" + distanceCenter);
            return;
        }

        // Calculate the change in heading
        double deltaTheta = (distanceRight - distanceLeft) / TRACK_WIDTH;

        // Update heading
        theta += Math.toDegrees(deltaTheta);
        theta = (theta + 360) % 360;

        // Calculate the average distance traveled forward
        double averageDistance = (distanceLeft + distanceRight) / 2.0;

        // Calculate change in position
        double deltaX = averageDistance * Math.cos(Math.toRadians(theta)) + distanceCenter * Math.sin(Math.toRadians(theta));
        double deltaY = averageDistance * Math.sin(Math.toRadians(theta)) - distanceCenter * Math.cos(Math.toRadians(theta));

        // Update position
        x += deltaX;
        y += deltaY * 1.046666666666666666666666666666666666666666666666667;
    }


    public static double getOdometryX() {
        return y;
    }

    public static double getOdometryY() {
        return x;
    }

    public static double getOdometryTheta() {
        return theta;
    }

    public static void setCurrentPositionAndResetEncoders(double fieldX, double fieldY, double theta) {
    }
    public static int getLeftEncoder(){
        return -backLeftMotor.getCurrentPosition();
    }
    public static int getRightEncoder(){
        return -backRightMotor.getCurrentPosition();
    }
    public static int getCenterEncoder(){
        return frontRightMotor.getCurrentPosition();
    }
    public static void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }
}
