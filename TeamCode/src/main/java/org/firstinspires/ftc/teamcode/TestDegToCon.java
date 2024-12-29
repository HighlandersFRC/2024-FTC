package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.ServoInputToDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.absoluteArmZero;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestDegToCon extends LinearOpMode {
    public double intakePosRight;
    public double intakePosLeft;

    public void runOpMode() throws InterruptedException {
        Servo RightIntake = hardwareMap.get(Servo.class, "IntakeRight");
        Servo LeftIntake = hardwareMap.get(Servo.class, "IntakeLeft");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakePosRight = 0;
                intakePosLeft = 0.5;
            } else if (gamepad1.b) {
                intakePosRight = 0.5;
                intakePosLeft = 0;
            }
            RightIntake.setPosition(intakePosRight);
            LeftIntake.setPosition(intakePosLeft);
        }
    }
}