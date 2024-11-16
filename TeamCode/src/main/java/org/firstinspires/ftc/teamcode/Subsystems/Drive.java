package org.firstinspires.ftc.teamcode.Subsystems;

import  com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.PID;

import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Drive extends Subsystem {

    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;

    public static final double TICKS_PER_REV = 2000;
    public static final double WHEEL_DIAMETER = 0.048; // meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double CORRECTION_FACTOR = 1;

    public static double x = 0.0;
    public static double y = 0.0;
    public static double theta = 0.0;

    public static double lastLeftPos = 0;
    public static double lastRightPos = 0;
    public static double lastCenterPos = 0;

    public static PID xPID = new PID(1, 0, 0);
    public static PID yPID = new PID(1, 0, 0);
    public static PID thetaPID = new PID(1, 0, 0);

    public static final double FORWARD_OFFSET = 0.22225;

    private static long lastUpdateTime = 0;

    public static double totalXTraveled = 0.0;
    public static double totalYTraveled = 0.0;
    public static double totalThetaTraveled = 0.0;
    private static final double L = 0.2;
    private static final double W = 0.2;

    public static void initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();
        lastUpdateTime = System.currentTimeMillis();
    }

    public static void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftMotor.setPower(-leftFrontPower);
        frontRightMotor.setPower(-rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(-rightBackPower);
    }

    public static void stop() {
    Drive.drive(0,0,0,0);

    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(
            DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Number[] purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
                                          JSONArray pathPoints) throws JSONException {
        JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
        int targetIndex = pathPoints.length() - 1;

        // Find the target point in the lookahead radius
        for (int i = currentIndex; i < pathPoints.length(); i++) {
            JSONObject point = pathPoints.getJSONObject(i);
            double pointX = point.getDouble("x");
            double pointY = point.getDouble("y");
            double pointTheta = point.getDouble("angle");

            // Normalize angle difference
            while (Math.abs(pointTheta - currentTheta) > Math.PI) {
                if (pointTheta - currentTheta > Math.PI) {
                    pointTheta -= 2 * Math.PI;
                } else if (pointTheta - currentTheta < -Math.PI) {
                    pointTheta += 2 * Math.PI;
                }
            }

            // Check if point is outside the lookahead radius
            if (!insideRadius(
                    (currentX - pointX) / Constants.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    (currentY - pointY) / Constants.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    (currentTheta - pointTheta) / Constants.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
                    Constants.AUTONOMOUS_LOOKAHEAD_DISTANCE
            )) {
                targetIndex = i;
                targetPoint = pathPoints.getJSONObject(i);
                break;
            }
        }

        double targetX = targetPoint.getDouble("x");
        double targetY = targetPoint.getDouble("y");
        double targetTheta = targetPoint.getDouble("angle");

        // Adjust angle difference again for targetTheta
        while (Math.abs(targetTheta - currentTheta) > Math.PI) {
            if (targetTheta - currentTheta > Math.PI) {
                targetTheta -= 2 * Math.PI;
            } else if (targetTheta - currentTheta < -Math.PI) {
                targetTheta += 2 * Math.PI;
            }
        }

        // Set target points for PID controllers
        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);

        // Update PID controllers based on current position
        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        // PID output without feedforward
        double xVelNoFF = xPID.getResult();
        double yVelNoFF = yPID.getResult();
        double thetaVelNoFF = -thetaPID.getResult();

        // Feedforward terms from velocity in path point
        double feedForwardX = targetPoint.getDouble("x_velocity") / 2;
        double feedForwardY = targetPoint.getDouble("y_velocity") / 2;
        double feedForwardTheta = -targetPoint.getDouble("angular_velocity") / 2;

        // Calculate final velocities with feedforward
        Number[] velocityArray = new Number[]{
                feedForwardX + xVelNoFF,
                -(feedForwardY + yVelNoFF),
                feedForwardTheta + thetaVelNoFF,
                targetIndex
        };

        return velocityArray;
    }
    private boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
    }

    public static double getVelocityBackLeft() {
        return backLeftMotor.getVelocity();
    }

    public static double getVelocityBackRight() {
        return backRightMotor.getVelocity();
    }

    public static double getVelocityFrontLeft() {
        return frontLeftMotor.getVelocity();
    }

    public static double getVelocityFrontRight() {
        return frontRightMotor.getVelocity();
    }

/*    public static void update() {
        double imuTheta = Peripherals.getYawDegrees();

        double currentLeftPos = getLeftEncoder();
        double currentRightPos = getRightEncoder();
        double currentCenterPos = getCenterEncoder();

        double deltaLeft = currentLeftPos - lastLeftPos;
        double deltaRight = currentRightPos - lastRightPos;
        double deltaCenter = currentCenterPos - lastCenterPos;

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastCenterPos = currentCenterPos;

        double distanceLeft = deltaLeft * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceRight = deltaRight * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceCenter = deltaCenter * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

        theta = imuTheta;

        double avgForwardMovement = (distanceLeft + distanceRight) / 2.0;
        double deltaTheta = Math.toRadians(theta);
        double deltaThetaDegrees = theta;

        double deltaX = avgForwardMovement * Math.cos(deltaTheta) + distanceCenter * Math.sin(deltaTheta);
        double deltaY = -avgForwardMovement * Math.sin(deltaTheta) + distanceCenter * Math.cos(deltaTheta);

        x += deltaX;
        y += deltaY;

        totalXTraveled += Math.abs(deltaX);
        totalYTraveled += Math.abs(deltaY);
    }*/

    public static void update() {
        double imuTheta = Peripherals.getYawDegrees();

        double currentLeftPos = getLeftEncoder();
        double currentRightPos = getRightEncoder();
        double currentCenterPos = getCenterEncoder();

        double deltaLeft = currentLeftPos - lastLeftPos;
        double deltaRight = currentRightPos - lastRightPos;
        double deltaCenter = currentCenterPos - lastCenterPos;

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastCenterPos = currentCenterPos;

        double distanceLeft = deltaLeft * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceRight = deltaRight * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceCenter = deltaCenter * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

        double avgForwardMovement = (distanceLeft + distanceRight) / 2.0;

        double deltaTheta = Math.toRadians(imuTheta - theta);
        theta = imuTheta;

        double thetaRadians = Math.toRadians(theta);

        double deltaX = avgForwardMovement * Math.cos(thetaRadians) - distanceCenter * Math.sin(thetaRadians);
        double deltaY = avgForwardMovement * Math.sin(thetaRadians) + distanceCenter * Math.cos(thetaRadians);

        x += deltaX;
        y += deltaY;

        x = Math.round(x * 1000) / 1000.0;
        y = Math.round(y * 1000) / 1000.0;

    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    public static double getLastLeftPos(){
        return lastLeftPos;
    }

    public static double getLastRightPos(){
        return lastRightPos;
    }

    public static double getLastCenterPos(){
        return lastCenterPos;
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

    public static double getTotalXTraveled() {
        return totalXTraveled;
    }

    public static double getTotalYTraveled() {
        return totalYTraveled;
    }

    public static double getTotalThetaTraveled() {
        return totalThetaTraveled;
    }

    public static void setPosition(double fieldX, double fieldY, double fieldTheta) {
        x = fieldX;
        y = fieldY;
        theta = fieldTheta;

        lastLeftPos = getLeftEncoder();
        lastRightPos = getRightEncoder();
        lastCenterPos = getCenterEncoder();
    }

    public static void resetEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        lastLeftPos = 0;
        lastRightPos = 0;
        lastCenterPos = 0;
    }


    public static int getLeftEncoder() {
        return backRightMotor.getCurrentPosition();
    }
    public static int getRightEncoder() {
        return frontLeftMotor.getCurrentPosition();
    }


 /*   public static void moveToAprilTag(int ID){
            double tagX = FieldOfMerit.x;
            double tagY = FieldOfMerit.y;
            double tagTheta = FieldOfMerit.tagyaw;
            PID turnPID = new PID(1, 0, 0);
            turnPID.setSetPoint(Peripherals.getYawDegrees() -Math.toDegrees(tagTheta));
            turnPID.updatePID(Peripherals.getYawDegrees());
            if (!(turnPID.getError() == 0)) {
                if (Math.abs(turnPID.getError()) < 2) {
                    double distance = Math.sqrt(x * x + y * y);
                    if ((distance < 0.1)){
                        Drive.drive(0.5,0.5,0.5,0.5);
                    }else {
                        Drive.stop();
                    }
                }else {
                    double power = turnPID.getResult();
                    Drive.drive(power, -power, power, -power);
                }

        }
    }*/

    public static int getCenterEncoder() {
        return frontRightMotor.getCurrentPosition();
    }
    public static void autoDrive(Vector vector, double omega) {
        // Tuneable parameters
        double velocityScale = 1.0; // Adjust overall speed
        double rotationScale = 1.0; // Adjust rotational impact
        double vx = vector.getJ() * velocityScale;
        double vy = -vector.getI() * velocityScale;

        double rotationFactor = omega * (L + W) * rotationScale;

        double frontLeftPower = vx + vy + rotationFactor;
        double frontRightPower = vx - vy - rotationFactor;
        double backLeftPower = vx - vy + rotationFactor;
        double backRightPower = vx + vy - rotationFactor;

        // Normalize powers if any exceeds the range [-1, 1]
        double maxMagnitude = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxMagnitude > 1) {
            frontLeftPower /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            backLeftPower /= maxMagnitude;
            backRightPower /= maxMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public static void setVelocities(double xVelocity, double yVelocity, double thetaVelocity) {
        Vector velocityVector = new Vector(yVelocity, xVelocity); // Adjust for coordinate system as in autoDrive
        autoDrive(velocityVector, thetaVelocity);
    }

}