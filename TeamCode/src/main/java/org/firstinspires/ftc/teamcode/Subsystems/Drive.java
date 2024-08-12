package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Drive extends Subsystem{
    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;

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
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"right_back");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();
    }

    public static void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);

        System.out.println("Drive Powers: FL=" + leftFrontPower + ", FR=" + rightFrontPower +
                ", BL=" + leftBackPower + ", BR=" + rightBackPower);
    }
    public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex, JSONArray pathPoints) throws JSONException {
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

    public static double getVelocityBackLeft(){
        return backLeftMotor.getVelocity();
    }
    public static double getVelocityBackRight(){
        return backRightMotor.getVelocity();
    }
    public static double getVelocityFrontLeft(){
        return frontLeftMotor.getVelocity();
    }
    public static double getVelocityFrontRight(){
        return frontRightMotor.getVelocity();
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
        return x;
    }

    public static double getOdometryY() {
        return y;
    }

    public static double getOdometryTheta() {
        return theta;
    }

    public static void setCurrentPositionAndResetEncoders(double fieldX, double fieldY, double theta) {
    }
    public static int getLeftEncoder(){
        return -frontLeftMotor.getCurrentPosition();
    }
    public static int getRightEncoder(){
        return -backRightMotor.getCurrentPosition();
    }
    public static int getCenterEncoder(){
        return frontRightMotor.getCurrentPosition();
    }
    public static void resetEncoder(){
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = 0;
        y = 0;
        theta = 0;
    }
}
