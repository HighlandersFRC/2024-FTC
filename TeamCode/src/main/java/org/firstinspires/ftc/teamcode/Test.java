package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Test extends LinearOpMode {

    private MotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;
    private ElapsedTime timer;

    public static final double TRACKWIDTH = 11;
    public static final double CENTER_WHEEL_OFFSET = 0;
    public static final double WHEEL_DIAMETER = 0.048;
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = new MotorEx(hardwareMap, "left_front");
        rightFrontMotor = new MotorEx(hardwareMap, "right_front");
        leftBackMotor = new MotorEx(hardwareMap, "left_back");
        rightBackMotor = new MotorEx(hardwareMap, "right_back");

        leftEncoder = rightBackMotor;
        rightEncoder = leftFrontMotor;
        perpEncoder = rightFrontMotor;

        leftEncoder.setInverted(true);

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        timer = new ElapsedTime();
        waitForStart();

        while (!isStopRequested()) {
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            mecanumDrive(x, y, rotation);

            telemetry.addData("X Position (m)", PositionTracker.robotPose.getX());
            telemetry.addData("Y Position (m)", PositionTracker.robotPose.getY());
            telemetry.addData("Heading (degrees)", Math.toDegrees(PositionTracker.robotPose.getHeading()));
            telemetry.addData("Left Encoder Distance", leftEncoder.getDistance());
            telemetry.addData("Right Encoder Distance", rightEncoder.getDistance());
            telemetry.addData("Perpendicular Encoder Distance", perpEncoder.getDistance());
            telemetry.update();
        }
    }

    private void mecanumDrive(double x, double y, double rotation) {
        double leftFrontPower = y + x + rotation;
        double rightFrontPower = y - x - rotation;
        double leftBackPower = y - x + rotation;
        double rightBackPower = y + x - rotation;

        double max = Math.abs(leftFrontPower);
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontMotor.set(-leftFrontPower);
        rightFrontMotor.set(rightFrontPower);
        leftBackMotor.set(-leftBackPower);
        rightBackMotor.set(-rightBackPower);
    }
}
