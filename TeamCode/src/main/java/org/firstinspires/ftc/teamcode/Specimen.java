package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Elevator;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Pivot1;
import org.firstinspires.ftc.teamcode.Commands.Pivot3;
import org.firstinspires.ftc.teamcode.Commands.PivotMove;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Commands.WristMove;
import org.firstinspires.ftc.teamcode.PathingTool.FirstPathFollower;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader0;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader2;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader3;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader4;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader5;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader6;
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
        PathLoader4 pathLoader4 = new PathLoader4(hardwareMap.appContext, "Autos/4Specimen.polarpath");
        PathLoader5 pathLoader5 = new PathLoader5(hardwareMap.appContext, "Autos/5Specimen.polarpath");
        PathLoader6 pathLoader6 = new PathLoader6(hardwareMap.appContext, "Autos/6Specimen.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive("drive");
        Peripherals peripherals = new Peripherals("peripherals");
        FirstPathFollower moveToPosition;
        PolarPathFollower move1;
        PolarPathFollower move2;
        PolarPathFollower move3;
        PolarPathFollower move4;
        PolarPathFollower move5;
        PolarPathFollower move6;



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
            move4 = new PolarPathFollower(drive,peripherals,pathLoader4.getJsonPathData(),Constants.commandMap,Constants.conditionMap,scheduler);
            move5 = new PolarPathFollower(drive,peripherals,pathLoader5.getJsonPathData(),Constants.commandMap,Constants.conditionMap,scheduler);
            move6 = new PolarPathFollower(drive,peripherals,pathLoader6.getJsonPathData(),Constants.commandMap,Constants.conditionMap,scheduler);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }


scheduler.schedule(new SequentialCommandGroup(scheduler,
        new Wait(1000),
        new WristMove(Robot.wrist, 1),
        moveToPosition,
        new Pivot3(Robot.pivot, Constants.ARM_HIGH),
        new WristMove(Robot.wrist, 0.48533192541184481),
        move1,
        new Pivot1(Robot.pivot,-10),
        move2,
        move3,
        new Elevator(Robot.elevators,930),
        new ParallelCommandGroup(scheduler, Parameters.ALL,
                move4,
                new WristMove(Robot.wrist, 0.15),
                new IntakeCommand(Robot.intake)
        ),
        new Elevator(Robot.elevators,0),
        new WristMove(Robot.wrist, 0.9),
        move5,
        new Pivot3(Robot.pivot, Constants.ARM_HIGH),
        new WristMove(Robot.wrist,0.48533192541184481),
        move6
        ));

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
            telemetry.addData("pivot", Pivot.getAngle());
            telemetry.update();
        }
    }
}
