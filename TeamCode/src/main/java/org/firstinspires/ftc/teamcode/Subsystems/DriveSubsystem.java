package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Tools.Odometry;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveSubsystem extends Subsystem {

    public static DcMotor leftMotor;
    public static DcMotor rightMotor;
    private static Telemetry telemetry;
    private static PID turnPID = new PID(1, 0, 0);
    private static PID drivePIDL = new PID(1, 0, 0);
    private static PID drivePIDR = new PID(1, 0, 0);

    private static final double COUNTS_PER_INCH = 30;

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

        telemetry = telemetry;

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static void moveToPosition(double angleDegrees, double targetX, double targetY) {
        double currentYaw = Peripherals.getYawDegrees();
        double angleError = angleDegrees - currentYaw;
        double currentX = Odometry.getX();
        double currentY = Odometry.getY();

        if (Math.abs(angleError) > 2) {
            turnPID.setSetPoint(angleDegrees);
            turnPID.updatePID(currentYaw);
            double turnPower = turnPID.getResult();

            Drive(turnPower, turnPower);

            System.out.println("Turning with power: " + turnPower + " Current Yaw: " + currentYaw + " Target Angle: " + angleDegrees);
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

                Drive(moveY + moveX, moveY - moveX);

                System.out.println("Moving with power: " + movePower + " Current Position: (" + currentX + ", " + currentY + ") Target Position: (" + targetX + ", " + targetY + ")");
            } else {
                stop();
            }
        }
    }


    private static double calculateRemainingDistance(double leftTargetPos, double rightTargetPos) {
        double currentLeftPos = leftMotor.getCurrentPosition();
        double currentRightPos = rightMotor.getCurrentPosition();

        return Math.abs(leftTargetPos - currentLeftPos) + Math.abs(rightTargetPos - currentRightPos);
    }

    public static void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private static void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static void setRunMode(DcMotor.RunMode mode) {
        leftMotor.setMode(mode);
        rightMotor.setMode(mode);
    }

    @Override
    public String getName() {
        return super.getName();
    }
}
