package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Strafe;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.json.JSONException;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals.initialize(hardwareMap);
/*
        HashMap<String, Supplier<Command>> commandMap = new HashMap<>();
        HashMap<String, BooleanSupplier> conditionMap = new HashMap<>();

        Drive.initialize(hardwareMap);
        Drive driveSubsystem = new Drive();
        Peripherals peripherals = new Peripherals("Peripherals");
        Peripherals.initialize(hardwareMap);

        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "OneMeter.polarpath");
        CommandScheduler scheduler = new CommandScheduler();*/

        Command polarFollow;
/*        try {
            polarFollow = new PolarPathFollower(driveSubsystem, peripherals, pathLoading.getJsonPathData(), commandMap, conditionMap, scheduler);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to initialize PolarPathFollower: " + e.getMessage());
            telemetry.update();
            return;
        }*/
        CommandScheduler scheduler = new CommandScheduler();
        polarFollow = new SequentialCommandGroup(scheduler, new Drive(hardwareMap, 1, 1, scheduler), new Strafe(hardwareMap, 1, 1, scheduler));

        waitForStart();

        try {
            scheduler.schedule(polarFollow);
        } catch (JSONException e) {
            telemetry.addData("Error", "Failed to schedule PolarPathFollower: " + e.getMessage());
            telemetry.update();
            return;
        }

        while (opModeIsActive()) {
            FieldOfMerit.processTags();
            try {
                scheduler.run();
                Thread.sleep(100);
            } catch (Exception e) {
                telemetry.update();
            }

            telemetry.addData("X", FinalPose.x);
            telemetry.addData("Y", FinalPose.y);
            telemetry.addData("Theta", Peripherals.getYawDegrees());
            telemetry.update();
        }
    }
}
