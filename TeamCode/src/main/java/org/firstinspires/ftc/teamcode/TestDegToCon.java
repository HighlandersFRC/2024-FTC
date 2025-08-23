package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class TestDegToCon extends LinearOpMode {

    DcMotor Elevator = hardwareMap.get(DcMotor.class, "pivotMotor");

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                Elevator.setPower(0.01);
            } else if (gamepad1.b) {
                Elevator.setPower(-0.01);
            }
            telemetry.update();
        }
    }
}
