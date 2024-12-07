package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tools.PID;

@TeleOp
public class MotorPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PID Test = new PID(0.05, 0.01, 0.01);
        DcMotor motor1;
        motor1 = hardwareMap.dcMotor.get("motor1");
        Test.setSetPoint(1000);

        waitForStart();
        while (opModeIsActive()){
            Test.updatePID(motor1.getCurrentPosition());
            motor1.setPower(gamepad1.left_stick_x);

            telemetry.addData("PID Result", Test.getResult());
            telemetry.addData("Motor Ticks", motor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
