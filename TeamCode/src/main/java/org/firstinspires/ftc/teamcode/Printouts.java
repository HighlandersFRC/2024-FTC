package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Simple TeleOp with Encoders", group = "TeleOp")
public class Printouts extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    @Override
    public void runOpMode() {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        // Set the motors to run without encoders (for teleop)
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Holonomic drive control
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Normalize powers if any exceed 1.0
            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;

            // Apply powers to the motors
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Display encoder values on telemetry
            telemetry.addData("Front Left Encoder", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Encoder", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Right Encoder", backRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
