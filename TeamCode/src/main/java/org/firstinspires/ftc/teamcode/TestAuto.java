package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;

@Autonomous(name="TestAuto", group="Autonomous")
public class TestAuto extends LinearOpMode {

    private PathEngine pathEngine;

    @Override
    public void runOpMode() {
        DriveSubsystem.start(hardwareMap);
        Peripherals.init(hardwareMap);
        // Load JSON file as an embedded resource
        try {
            InputStream inputStream = getClass().getResourceAsStream("/ParabolicPath.json");
            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            StringBuilder jsonBuilder = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
                jsonBuilder.append('\n');
            }
            String jsonString = jsonBuilder.toString();

            // Initialize PathEngine with loaded JSON data and hardwareMap
            pathEngine = new PathEngine(jsonString, hardwareMap);

        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addData("Error", "Failed to load path JSON.");
            telemetry.update();
            sleep(3000);
            return;
        }

        // Wait for the start button to be pressed
        waitForStart();

        // Autonomous loop
        while (opModeIsActive()) {
            telemetry.addData("L + R Encoders", DriveSubsystem.getLeftEncoder() + "   " + DriveSubsystem.getRightEncoder());
            // Update the path engine to follow the path
            pathEngine.update();

            // Example: Stop op mode when path is completed

            telemetry.update();
        }
    }
}
