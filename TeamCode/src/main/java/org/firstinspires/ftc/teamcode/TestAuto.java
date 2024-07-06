package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.json.JSONException;

import java.io.IOException;

@Autonomous(name="TestAuto", group="Linear Opmode")

public class TestAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = new DriveSubsystem("drive", hardwareMap);
        Peripherals.initialize(hardwareMap);
        PathEngine pathEngine = new PathEngine(this, "ParabolicPath.json");
        waitForStart();
        while (opModeIsActive()){
            pathEngine.update();
            sleep(50);
        }
    }
}
