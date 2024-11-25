package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.json.JSONException;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        Mouse.configureOtos();

        Drive.setPosition(0, 0, 0);

        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Thing.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive();
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower moveToPosition;

 /*       try {
            scheduler.schedule(new Wait(3000));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }*/

        try {
            moveToPosition = new PolarPathFollower(drive, peripherals, PathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            scheduler.schedule(moveToPosition);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        while (opModeIsActive()) {
            FinalPose.poseUpdate();
                        try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.Yaw;

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.update();
        }
    }
}
