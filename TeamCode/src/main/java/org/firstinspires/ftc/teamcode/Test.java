package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Vector;


@TeleOp
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive("Drive", hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Vector vector = new Vector(10, 10);
            drive.autoDrive(vector, 0);
        }
    }
}
