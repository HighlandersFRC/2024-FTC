package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;


@TeleOp
public class TankDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DriveSubsystem.initialize(hardwareMap);
        while (opModeIsActive()){

            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            double intakePower = leftTrigger - rightTrigger ;

            double denominator = Math.max(Math.abs(y) + Math.abs(rx), 1);
            double leftPower = (y + rx) / denominator;
            double rightPower = (y - rx) / denominator;
            DriveSubsystem.Drive(leftPower, rightPower);

        }
    }
}