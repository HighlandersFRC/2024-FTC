package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp
public class TestClass extends LinearOpMode {
    ElevatorSubsystem elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        elevator = new ElevatorSubsystem("Elevator", hardwareMap);

        waitForStart();
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            elevator.manual(gamepad1);
        } else {
            elevator.setPosition(-150);
        }

        while (opModeIsActive()) {
            elevator.manual(gamepad1);
            telemetry.addData("Cur Pos", elevator.getCurrentPosition());
            telemetry.update();
        }
    }
}
