package org.firstinspires.ftc.teamcode;

import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.json.JSONException;

import java.io.IOException;

@Autonomous(name="TestAuto", group="Linear Opmode")
@Disabled
public class TestAuto extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private PathEngine pathEngine;

    @Override
    public void runOpMode() {
        driveSubsystem = new DriveSubsystem("DriveSubsystem", hardwareMap);
        pathEngine = new PathEngine("ParabolicPath.json", driveSubsystem, telemetry, this);

        waitForStart();

        try {
            pathEngine.initializePath();
            pathEngine.runPathFollowing();
        } catch (IOException | InterruptedException | JSONException e) {
            telemetry.addData("Error", "Failed to load or execute path");
            telemetry.update();
            sleep(2000);
            return;
        }
    }

    public AssetManager getAssets() {
        return assets;
    }
}
