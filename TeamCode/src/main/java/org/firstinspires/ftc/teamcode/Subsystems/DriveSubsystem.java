package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Tools.Odometry;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveSubsystem {

    public static DcMotor leftBack, leftFront, rightBack, rightFront;
    private PID turnPID = new PID(1, 0, 0);
    private PID drivePIDL = new PID(1, 0, 0);
    private PID drivePIDR = new PID(1, 0, 0);

    private static final double COUNTS_PER_INCH = 30;

    public DriveSubsystem(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    public static void initialize(HardwareMap hardwareMap) {
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();}

    public static void MecanumDrive(double x, double y, double rx) {
        double botHeading = Peripherals.getYaw();
        double pi = Math.PI;
        double botHeadingRadian = -botHeading * pi/180;

        double rotX = (x * Math.cos(botHeadingRadian) - y * Math.sin(botHeadingRadian));
        double rotY = (x * Math.sin(botHeadingRadian) + y * Math.cos(botHeadingRadian));

        x = x *1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public static void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
/*

    private double calculateRemainingDistance(double leftTargetPos, double rightTargetPos) {
        double currentLeftPos = leftMotor.getCurrentPosition();
        double currentRightPos = rightMotor.getCurrentPosition();

        return Math.abs(leftTargetPos - currentLeftPos) + Math.abs(rightTargetPos - currentRightPos);
    }
*/

    /*public void moveToPosition(double angleDegrees, double targetX, double targetY) {
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
    }*/

    public void stop() {
        drive(0, 0, 0, 0);
    }

    private static void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private static void setRunMode(DcMotor.RunMode mode) {
        leftBack.setMode(mode);
        rightBack.setMode(mode);
        leftFront.setMode(mode);
        rightFront.setMode(mode);
    }
}
