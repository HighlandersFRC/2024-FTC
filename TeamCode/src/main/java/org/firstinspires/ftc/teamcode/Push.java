package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;

@Autonomous
public class Push extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Pivot.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Elevators.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);

        Mouse.configureOtos();
        Drive.setPosition(0, 0, 0);


        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Autos/Push.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive("drive");
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower moveToPosition;


 /*       try {9[
            scheduler.schedule(new Wait(3000));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }*/



        waitForStart();

        try {
            moveToPosition = new PolarPathFollower(drive, peripherals, pathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            scheduler.schedule(moveToPosition);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {

            FinalPose.poseUpdate();

            scheduler.run();

            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.yaw;

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.update();

        }
    }
}
