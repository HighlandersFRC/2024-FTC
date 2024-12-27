package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class Test extends LinearOpMode {
    public static double pos;
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive("drive",hardwareMap,telemetry);
        drive.initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            drive.FeildCentric(gamepad1);
            Mouse.update();

            telemetry.update();
        }
    }
}