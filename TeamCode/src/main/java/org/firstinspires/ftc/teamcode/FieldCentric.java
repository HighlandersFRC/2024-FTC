package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive.initialize(hardwareMap);
        Peripherals.initialize(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            Drive.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            telemetry.addData("OdoX", Drive.getOdometryX());
            telemetry.addData("OdoY", Drive.getOdometryY());
            telemetry.addData("OdoThetha", Drive.getOdometryTheta());
            telemetry.addData("IMU Yaw", Peripherals.getYawDegrees());
            telemetry.addData("Left Encoder", Drive.getLeftEncoder());
            telemetry.addData("Right Encoder", Drive.getRightEncoder());
            telemetry.addData("Center Encoder", Drive.getCenterEncoder());
            telemetry.update();
            Drive.update();
        }
    }
}