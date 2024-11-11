package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot systems
        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Mouse.update();
            FinalPose.poseUpdate();

            // Retrieve gamepad inputs
            double forward = -gamepad1.left_stick_y; // forward and backward
            double strafe = gamepad1.left_stick_x;   // left and right
            double rotation = gamepad1.right_stick_x; // rotation

            // Reset yaw and encoders if the right bumper is pressed
            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
                Mouse.configureOtos();
            }
            // Get robot's current heading in radians
            double botHeading = Math.toRadians(Mouse.getTheta());

            // Rotate joystick inputs to field-centric coordinates
            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            // Calculate the raw motor powers using your specified logic
            double frontLeftPower = rotY + rotX + rotation;
            double frontRightPower = rotY - rotX - rotation;
            double backLeftPower = rotY - rotX + rotation;
            double backRightPower = rotY + rotX - rotation;

            // Find the maximum absolute power value among the motors
            double maxPower = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))
            ));

            // Scale all motor powers down proportionally if any exceed 1.0
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;

            // Drive the motors
            Drive.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Telemetry display for debugging
            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());
            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());
            telemetry.addLine("Mouse Sensor Values")
                    .addData("X (m)", "%.2f", Mouse.getX())
                    .addData("Y (m)", "%.2f", Mouse.getY())
                    .addData("Theta (deg)", "%.2f", Mouse.getTheta());
            telemetry.addLine("Fused Pose")
                    .addData("X (m)", "%.2f", FinalPose.x)
                    .addData("Y (m)", "%.2f", FinalPose.y)
                    .addData("Theta (deg)", "%.2f", FinalPose.Yaw);
            telemetry.addLine("Encoders")
                    .addData("Left Encoder", Drive.getLeftEncoder())
                    .addData("Right Encoder", Drive.getRightEncoder())
                    .addData("Center Encoder", Drive.getCenterEncoder());
            telemetry.addLine("Sensors")
                    .addData("Current Sensor State", FieldOfMerit.currentState);
            telemetry.update();
        }
    }
}
