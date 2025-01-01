package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestDegToCon extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        // Set motors to STOP_AND_RESET_ENCODER and then RUN_USING_ENCODER mode
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start button to be pressed
        waitForStart();

        // Calculate encoder ticks for 90-degree turn
        double turn90 = DegreesToEncoderTicks(90);

        // Set the setpoint for the PID controller
        piviotPID.setSetPoint(turn90);

        // Configure PID output limits
        piviotPID.setMinOutput(-1);
        piviotPID.setMaxOutput(1);

        // Update PID with the current positions of each motor and apply the calculated power
        while (opModeIsActive()) {
            double frontLeftOutput = piviotPID.updatePID(frontLeftMotor.getCurrentPosition());
            double frontRightOutput = piviotPID.updatePID(frontRightMotor.getCurrentPosition());
            double backLeftOutput = piviotPID.updatePID(backLeftMotor.getCurrentPosition());
            double backRightOutput = piviotPID.updatePID(backRightMotor.getCurrentPosition());

            frontLeftMotor.setPower(-frontLeftOutput);
            frontRightMotor.setPower(-frontRightOutput);
            backLeftMotor.setPower(-backLeftOutput);
            backRightMotor.setPower(-backRightOutput);

            telemetry.addData("front left motor", frontLeftMotor.getCurrentPosition());
            telemetry.addData("front right motor", frontRightMotor.getCurrentPosition());
            telemetry.addData("back left motor", backLeftMotor.getCurrentPosition());
            telemetry.addData("back right motor", backRightMotor.getCurrentPosition());
            telemetry.addData("Front Left Output", frontLeftOutput);
            telemetry.addData("Front Right Output", frontRightOutput);
            telemetry.addData("Back Left Output", backLeftOutput);
            telemetry.addData("Back Right Output", backRightOutput);
            telemetry.addData("Target Position", turn90);
            telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Position", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right Position", backRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
