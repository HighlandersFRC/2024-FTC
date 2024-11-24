/*

package org.firstinspires.ftc.teamcode.Tools;

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
        Robot.initialize(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            FinalPose.poseUpdate();

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }


            if (gamepad1.a) {
                Drive.drive(1, 0, 0, 0);
            }
            if (gamepad1.b) {
                Drive.drive(0, 1, 0, 0);
            }
            if (gamepad1.y) {
                Drive.drive(0, 0, 1, 0);
            }
            if (gamepad1.x) {
                Drive.drive(0, 0, 0, 1);
            }


            double botHeading = Peripherals.getYaw();


            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            rotX *= 1.1;


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;




            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);


            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());

            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());



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

}*/
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot.initialize(hardwareMap);
        Drive.initialize(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            FinalPose.poseUpdate();


            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;


            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = FinalPose.botHeading;




            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);


            double frontLeftPower = rotY + rotX + rotation;
            double frontRightPower = rotY - rotX - rotation;
            double backLeftPower = rotY - rotX + rotation;
            double backRightPower = rotY + rotX - rotation;

            double maxPower = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))
            ));


            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;


            Drive.drive(-frontLeftPower, -frontRightPower, backLeftPower, -backRightPower);

            if (gamepad1.a) {
                Drive.drive(1, 0, 0, 0);
            }
            if (gamepad1.b) {
                Drive.drive(0, 1, 0, 0);
            }
            if (gamepad1.y) {
                Drive.drive(0, 0, 1, 0);
            }
            if (gamepad1.x) {
                Drive.drive(0, 0, 0, 1);
            }
            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());
            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());
            telemetry.addLine("Mouse Sensor Values")
                    .addData("X (m)", "%.2f", Peripherals.getOtosX())
                    .addData("Y (m)", "%.2f", Peripherals.getOtosY())
                    .addData("Theta (deg)", "%.2f", Peripherals.getYawDegrees());
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