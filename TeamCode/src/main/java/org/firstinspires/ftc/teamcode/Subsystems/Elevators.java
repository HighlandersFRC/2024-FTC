package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.PID;

public class Elevators extends Subsystem {
    private static DcMotor leftElevator;
    private static DcMotor rightElevator;
    private static PID leftElevatorPID = new PID(0.3, 0.0, 0.0);
    private static PID rightElevatorPID = new PID(0.3, 0.0, 0.0);

    private static final int STALL_THRESHOLD = 10; // Encoder ticks below this are considered no movement
    private static final long MONITOR_INTERVAL_MS = 100; // Time between encoder checks in milliseconds

    public static void initialize(HardwareMap hardwareMap) {
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        resetEncoders();
        setBrakeMode();
    }

    public static void moveUp() {
        setPower(1.0);
    }

    public static void moveDown() {
        setPower(-0.4);
        monitorDownMovement();
    }

    public static void moveLeftElevator(double power) {
        leftElevator.setPower(power);
    }

    public static void moveRightElevator(double power) {
        rightElevator.setPower(power);
    }

    public static void stop() {
        setPower(0.0);
    }

    public static void resetEncoders() {
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double getLeftEncoder() {
        return leftElevator.getCurrentPosition();
    }

    public static double getRightEncoder() {
        return rightElevator.getCurrentPosition();
    }

    private static void setPower(double power) {
        leftElevator.setPower(power);
        rightElevator.setPower(power);
    }

    private static void setBrakeMode() {
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void monitorDownMovement() {
        new Thread(() -> {
            double initialLeftPosition = getLeftEncoder();
            double initialRightPosition = getRightEncoder();

            try {
                Thread.sleep(MONITOR_INTERVAL_MS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            double currentLeftPosition = getLeftEncoder();
            double currentRightPosition = getRightEncoder();

            boolean leftStalled = Math.abs(currentLeftPosition - initialLeftPosition) < STALL_THRESHOLD;
            boolean rightStalled = Math.abs(currentRightPosition - initialRightPosition) < STALL_THRESHOLD;

            if (leftStalled || rightStalled) {
                stop();
            }
        }).start();
    }
}
