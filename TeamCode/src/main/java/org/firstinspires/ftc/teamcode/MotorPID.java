package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.NewDegreesToEncoderTicks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tools.PID;

@TeleOp
public class MotorPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PID Test = new PID(0.035, 0.0, 0.04);
        DcMotor motor1;
        motor1 = hardwareMap.dcMotor.get("motor1");
        Test.setSetPoint(NewDegreesToEncoderTicks(1800));

        waitForStart();

        while (opModeIsActive()){
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Test.updatePID(motor1.getCurrentPosition());
                motor1.setPower(Test.getResult());

            telemetry.addData("PID Result", Test.getResult());
            telemetry.addData("Motor Ticks", motor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
