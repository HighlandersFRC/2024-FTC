package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveSubsystem extends Subsystem {

    public static DcMotor leftMotor;
    public static DcMotor rightMotor;
    private static Telemetry telemetry;
    private static PID turnPID = new PID(1, 0, 0);
    private static PID drivePIDL = new PID(1, 0, 0);
    private static PID drivePIDR = new PID(1, 0, 0);

    private static final double COUNTS_PER_INCH = 9999; // Example value, adjust for your robot

    public DriveSubsystem(String name, HardwareMap hardwareMap) {
        super(name);
        initialize(hardwareMap);
    }

    public static void Drive(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public static void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        // Initialize telemetry here or pass it from OpMode
        telemetry = telemetry;

        // Set motor directions and other configurations as needed
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders if needed
        resetEncoders();

        // Set motor run modes
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveToPosition(double x, double y, double distanceThreshold, double angleDegrees, double leftTargetPos, double rightTargetPos) {

        double leftSpeed = 0;
        double rightSpeed = 0;

        double remainingDistance = calculateRemainingDistance(leftTargetPos, rightTargetPos);
        {

            if (Math.abs(turnPID.getError()) < 2) {
                drivePIDL.setSetPoint(leftTargetPos);
                drivePIDR.setSetPoint(rightTargetPos);

                drivePIDL.clamp(1);
                drivePIDR.clamp(1);

                drivePIDL.updatePID(leftMotor.getCurrentPosition());
                drivePIDR.updatePID(rightMotor.getCurrentPosition());

                leftSpeed = drivePIDL.getResult();
                rightSpeed = drivePIDR.getResult();
                System.out.println(leftSpeed);

            } else {

                    turnPID.setSetPoint(angleDegrees);
                    turnPID.updatePID(Peripherals.getYawDegrees());
                    turnPID.setPID(0.03, 0, 0);
                    turnPID.clamp(1);

                    leftSpeed = turnPID.getResult();
                    rightSpeed = -turnPID.getResult();
                    System.out.println(leftSpeed);

            }
        }


        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {
        }
    }

    private static double calculateRemainingDistance(double leftTargetPos, double rightTargetPos) {
        double currentLeftPos = leftMotor.getCurrentPosition();
        double currentRightPos = rightMotor.getCurrentPosition();

        return Math.abs(leftTargetPos - currentLeftPos) + Math.abs(rightTargetPos - currentRightPos);
    }

    public static void stop() {
        // Stop both motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private static void resetEncoders() {
        // Reset motor encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunMode(DcMotor.RunMode mode) {
        // Set motor run mode
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    @Override
    public String getName() {
        return super.getName();
    }
}
