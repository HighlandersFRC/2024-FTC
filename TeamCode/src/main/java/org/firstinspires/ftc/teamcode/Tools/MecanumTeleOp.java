package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        CRServo wristServo = hardwareMap.get(CRServo.class, "wrist_servo");
        DcMotor armMotor = hardwareMap.dcMotor.get("arm_Motor");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        if (isStopRequested()) return null;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.a) {
                intakeServo.setPower(1.0);
            } else if (gamepad1.b) {
                intakeServo.setPower(-1.0);
            } else {
                intakeServo.setPower(0);
            }

            if (gamepad1.x) {
                wristServo.setPower(1.0);
            } else if (gamepad1.y) {
                wristServo.setPower(-1.0);
            } else {
                wristServo.setPower(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                armMotor.setPower(0.3);
            } else if (gamepad1.right_trigger > 0.1) {
                armMotor.setPower(-0.3);
            } else {
                armMotor.setPower(0);
            }
        }
        return null;
    }
}
