package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;

@TeleOp
public class TestClass extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystem armSubsystem = new ArmSubsystem("arm", hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
                armSubsystem.ArmMovement(gamepad1);


            telemetry.update();
        }
    }
}
