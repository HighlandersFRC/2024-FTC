package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.encodersToDeg;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;


@TeleOp
public class TestDegToCon extends LinearOpMode {
    public static double pos;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ArmSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {


            if(gamepad1.touchpad_finger_2) {
                telemetry.addData("What", "It worked");
                gamepad1.rumble(1000);
            } else {
                telemetry.addData("It" , "failed");
            }
            telemetry.update();

        }
    }
}