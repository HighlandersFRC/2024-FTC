package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class FieldCentricBackUp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
       DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
       DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
       DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
       DcMotor backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_x*2;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;



            double frontLeftPower = (-y + x + rx);
            double backLeftPower = (y + x - rx);
            double frontRightPower = (y + x + rx);
            double backRightPower = (y - x + rx);


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
