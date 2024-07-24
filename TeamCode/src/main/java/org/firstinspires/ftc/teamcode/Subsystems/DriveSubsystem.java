package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Odometry;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveSubsystem {

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private PID turnPID = new PID(1, 0, 0);
    private PID drivePIDL = new PID(1, 0, 0);
    private PID drivePIDR = new PID(1, 0, 0);

    private static final double COUNTS_PER_INCH = 30;

    public DriveSubsystem(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    public static void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();}

    public static void drive(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void moveToPosition(double angleDegrees, double targetX, double targetY) {
        double currentYaw = Odometry.getTheta(); // Replace with actual yaw retrieval method
        double angleError = angleDegrees - currentYaw;
        double currentX = Odometry.getX();
        double currentY = Odometry.getY();

        if (Math.abs(angleError) > 2) {
            turnPID.setSetPoint(angleDegrees);
            turnPID.updatePID(currentYaw);
            double turnPower = turnPID.getResult();

            drive(turnPower, turnPower);
        } else {
            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double distanceError = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distanceError > 0.1) {
                drivePIDL.setSetPoint(distanceError);
                drivePIDL.updatePID(0);
                double movePower = drivePIDL.getResult();

                double angleRadians = Math.toRadians(angleDegrees);
                double moveX = movePower * Math.cos(angleRadians);
                double moveY = movePower * Math.sin(angleRadians);

                drive(moveY + moveX, moveY - moveX);
            } else {
                stop();
            }
        }
    }

    private double calculateRemainingDistance(double leftTargetPos, double rightTargetPos) {
        double currentLeftPos = leftMotor.getCurrentPosition();
        double currentRightPos = rightMotor.getCurrentPosition();

        return Math.abs(leftTargetPos - currentLeftPos) + Math.abs(rightTargetPos - currentRightPos);
    }

    public void stop() {
        drive(0, 0);
    }

    private static void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }
}
