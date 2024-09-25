package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class LimelightTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initialize(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;


    }



}

