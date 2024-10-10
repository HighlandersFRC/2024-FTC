package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot systems
        Robot.initialize(hardwareMap);
        SparkFunOTOS myOtos;
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update the robot's pose
            FinalPose.poseUpdate();

            // Retrieve gamepad inputs
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset the yaw and encoders if the right bumper is pressed
            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            //debug stuff
            if (gamepad1.a) {
                Drive.drive(1,0,0,0);
            }
            if (gamepad1.b) {
                Drive.drive(0,1,0,0);
            }
            if (gamepad1.y){
                Drive.drive(0,0,1,0);
            }
            if (gamepad1.x){
                Drive.drive(0,0,0,1);
            }



            // Get robot's current heading
            double botHeading = -Peripherals.getYaw();

            // Rotate gamepad inputs to align with the field
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Slightly boost the strafe power
            rotX *= 1.1;

            // Normalize motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Drive the motors
            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

            // Clean telemetry
            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());

            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());

            telemetry.addLine("Mouse Sensor Values")
                    .addData("X (m)", "%.2f", Drive.getTotalXTraveled())
                    .addData("Y (m)", "%.2f", Drive.getTotalYTraveled())
                    .addData("Theta (deg)", "%.2f", Drive.totalThetaTraveled);

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

