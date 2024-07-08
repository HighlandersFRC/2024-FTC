package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.calculateRemainingDistance;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.drivePIDL;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.turnPID;

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
           System.out.println("PidError"+ drivePIDL.getError());
           System.out.println("PidPower"+ drivePIDL.getResult());
           System.out.println("TurnPidError"+turnPID.getError());
           System.out.println("JsonX"+ pathEngine.getNextSampledPointX());
           System.out.println("JsonY"+ pathEngine.getNextSampledPointY());
           System.out.println("JsonAngle"+ pathEngine.getNextSampledPointAngle());
           System.out.println("current posL"+ DriveSubsystem.CurrentPosL());
            System.out.println("current posR"+ DriveSubsystem.CurrentPosR());
            pathEngine.update();
            sleep(50);

        }
    }
}
