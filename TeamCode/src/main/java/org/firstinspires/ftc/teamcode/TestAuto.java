package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PathingTool.PathEngine;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

@Autonomous(name = "Path Following Autonomous", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    private PathEngine pathEngine;
    private CommandScheduler scheduler;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        Robot.initialize(hardwareMap);

        scheduler = CommandScheduler.getInstance();

        pathEngine = new PathEngine(this, "SimplePath.polarpath");

        waitForStart();

        runtime = new ElapsedTime();

        pathEngine.startPath(runtime.time());

        while (opModeIsActive()) {
            try {
                pathEngine.update(runtime.time());

                scheduler.run();
            } catch (InterruptedException | JSONException e) {

                e.printStackTrace();
            }

            telemetry.update();
        }

        scheduler.cancelAll();
    }
}
