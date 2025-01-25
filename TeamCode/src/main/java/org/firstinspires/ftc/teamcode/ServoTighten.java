package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTighten extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        CRServo RightIntake = hardwareMap.get(CRServo.class, "IntakeRight");
        CRServo LeftIntake = hardwareMap.get(CRServo.class, "IntakeLeft");
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                RightIntake.setPower(1);
            } else if (gamepad1.b) {
                RightIntake.setPower(-1);
            } else {
                RightIntake.setPower(0);
            }

            if (gamepad1.x) {
                LeftIntake.setPower(1);
            } else if (gamepad1.y) {
                LeftIntake.setPower(-1);
            } else {
                LeftIntake.setPower(0);
            }
            telemetry.update();
        }
    }
}
