package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Elevator;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Pivot1;
import org.firstinspires.ftc.teamcode.Commands.PivotMove;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Commands.WristMove;
import org.firstinspires.ftc.teamcode.PathingTool.FirstPathFollower;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader0;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader2;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader3;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

import java.security.Permissions;

@Autonomous
public class Specimen extends LinearOpMode {

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


        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Autos/Specimen.polarpath");
        PathLoader0 pathLoader0 = new PathLoader0(hardwareMap.appContext, "Autos/1Specimen.polarpath");
        PathLoader2 pathLoader2 = new PathLoader2(hardwareMap.appContext, "Autos/2Specimen.polarpath");
        PathLoader3 pathLoader3 = new PathLoader3(hardwareMap.appContext, "Autos/3Specimen.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive("drive");
        Peripherals peripherals = new Peripherals("peripherals");
        FirstPathFollower moveToPosition;
        PolarPathFollower move1;
        PolarPathFollower move2;
        PolarPathFollower move3;



 /*       try {9[
            scheduler.schedule(new Wait(3000));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }*/

        waitForStart();

        try {
            moveToPosition = new FirstPathFollower(drive, peripherals, pathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            move1 = new PolarPathFollower(drive, peripherals, pathLoader0.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            move2 = new PolarPathFollower(drive,peripherals, pathLoader2.getJsonPathData(),Constants.commandMap,Constants.conditionMap,scheduler);
            move3 = new PolarPathFollower(drive,peripherals,pathLoader3.getJsonPathData(),Constants.commandMap,Constants.conditionMap,scheduler);


        } catch (Exception e) {
            throw new RuntimeException(e);
        }


scheduler.schedule(new SequentialCommandGroup(scheduler, new Wait(1000), new WristMove(Robot.wrist, 1), moveToPosition /*new PivotMove(Robot.pivot, Constants.ARM_HIGH),*//* new WristMove(Robot.wrist, 0.1)*/, move1,/*new Pivot1(Robot.pivot,-10),*/move2,move3));

        while (opModeIsActive()) {

            FinalPose.poseUpdate();



            scheduler.run();



            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.yaw;

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData("Current State", FieldOfMerit.currentState);
            telemetry.update();
        }
    }
}
