package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Robot.elevators;
import static org.firstinspires.ftc.teamcode.Tools.Robot.pivot;
import static org.firstinspires.ftc.teamcode.Tools.Robot.wrist;

import android.graphics.Path;
import android.provider.ContactsContract;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.PathingTool.FirstPathFollower;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader0;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader2;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader3;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader4;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoader5;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Tools.*;


@Autonomous
public class TestAuto extends LinearOpMode {


    private FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();


        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Pivot.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Elevators.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);

        Robot.CURRENT_STATE = "Auto";


        Mouse.configureOtos();


        PathLoader0 pathLoader0 = new PathLoader0(hardwareMap.appContext, "Autos/0Preload.polarpath");
        PathLoader2 pathLoading = new PathLoader2(hardwareMap.appContext, "Autos/1Preload.polarpath");
        PathLoading path2 = new PathLoading(hardwareMap.appContext, "Autos/2Preload.polarpath");
        PathLoader3 paththree = new PathLoader3(hardwareMap.appContext, "Autos/3Preload.polarpath");
        PathLoader4 pathfour = new PathLoader4(hardwareMap.appContext, "Autos/4Preload.polarpath");
        PathLoader5 pathfive = new PathLoader5(hardwareMap.appContext, "Autos/5Preload.polarpath");




        CommandScheduler scheduler = new CommandScheduler();
        org.firstinspires.ftc.teamcode.Subsystems.Drive drive = new org.firstinspires.ftc.teamcode.Subsystems.Drive("drive");
        Peripherals peripherals = new Peripherals("peripherals");
        FirstPathFollower path0;
        PolarPathFollower path1;
        PolarPathFollower path2command;
        PolarPathFollower path3;
        PolarPathFollower path4;
        PolarPathFollower path5;




        waitForStart();


        try {
            path0 = new FirstPathFollower(drive, peripherals, pathLoader0.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            path1 = new PolarPathFollower(drive, peripherals, pathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            path2command = new PolarPathFollower(drive, peripherals, path2.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            path3 = new PolarPathFollower(drive, peripherals, paththree.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            path4 = new PolarPathFollower(drive, peripherals, pathfour.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            path5 = new PolarPathFollower(drive, peripherals, pathfive.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);

            Command place = new SequentialCommandGroup(scheduler,
                    new Pivot3(Robot.pivot, Constants.ARM_HIGH),
                    new Elevator(Robot.elevators, Constants.ELEVATOR_AUTO));

            Command place2 = new SequentialCommandGroup(scheduler,
                    new Pivot3(Robot.pivot, Constants.ARM_HIGH),
                    new Elevator(Robot.elevators, Constants.ELEVATOR_AUTO),
                    new WristMove(wrist, 0.6 - Constants.WRIST_OFFSET));
                    new ParallelCommandGroup(scheduler, Parameters.ANY,
                        new Outtake(Robot.intake, 2000),
                         new Wait(2000)
            );

            Command place3 = new SequentialCommandGroup(scheduler,
                    new PivotMove(Robot.pivot, Constants.ARM_HIGH),
                    new Elevator(Robot.elevators, Constants.ELEVATOR_AUTO));

            Command reset = new SequentialCommandGroup(scheduler,
                    new WristMove(wrist, 0.1 - Constants.WRIST_OFFSET),
                    new Elevator(Robot.elevators, 0),
                    new WristMove(wrist, 1 - Constants.WRIST_OFFSET),
                    new Pivot1(Robot.pivot, -10)
            );


            scheduler.schedule(new SequentialCommandGroup(scheduler,
                    new WristMove(Robot.wrist, 0.2 - Constants.WRIST_OFFSET),
                    path0,
                    place,
                    new WristMove(wrist, 0.4 - Constants.WRIST_OFFSET),
                    new Wait(1000),
                    new ParallelCommandGroup(scheduler, Parameters.ANY,
                            new OuttakeSpecify(Robot.intake, 1000, 1, 0.6),
                            new Wait(1000)
                    ),
                    reset,
                    new ParallelCommandGroup(scheduler, Parameters.ALL,
                            path1,
                            new WristMove(Robot.wrist, 0.1 - Constants.WRIST_OFFSET),
                            new IntakeCommand(Robot.intake)
                    ),
                    path2command,
                    new WristMove(Robot.wrist, 0.3 - Constants.WRIST_OFFSET),
                    place3,
                    new WristMove(wrist, 0.4 - Constants.WRIST_OFFSET),
                    new ParallelCommandGroup(scheduler, Parameters.ANY,
                            new Outtake(Robot.intake, 500),
                            new Wait(500)
                    ),
                    reset,
                    new ParallelCommandGroup(scheduler, Parameters.ALL,
                            path3,
                            new WristMove(Robot.wrist, 0.1 - Constants.WRIST_OFFSET),
                            new IntakeCommand(Robot.intake)
                    ),
                    path4,
                    place3,
                    new WristMove(wrist, 0.4 - Constants.WRIST_OFFSET),
                    new ParallelCommandGroup(scheduler, Parameters.ANY,
                            new Outtake(Robot.intake, 500),
                            new Wait(500)
                    ),
                    new ParallelCommandGroup(scheduler, Parameters.ANY,
                            new Outtake(Robot.intake, 500),
                            new Wait(500)
                    ),
                    new ParallelCommandGroup(scheduler, Parameters.ALL, new Elevator(elevators, 500)),
                    new WristMove(wrist, 0.5 - Constants.WRIST_OFFSET),
                    new ParallelCommandGroup(scheduler, Parameters.ALL,
                    new Elevator(elevators, 0),
                  /*  path5,*/
                    new SequentialCommandGroup(scheduler, new Wait(500),
                    new Pivot1(pivot, -16)
                  /*  new Elevator(elevators, 500)*/
            ))));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }


        while (opModeIsActive()) {
            RobotLog.d(Elevators.getLeftEncoder() + " " + Elevators.getRightEncoder());

            FinalPose.poseUpdate();
            scheduler.run();

            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = Math.toRadians(FinalPose.yaw);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldCanvas = packet.fieldOverlay();

            fieldCanvas.setStroke("red");
            fieldCanvas.fillCircle(robotX, robotY, 3);

            double arrowX = robotX + 6 * Math.cos(robotTheta);
            double arrowY = robotY + 6 * Math.sin(robotTheta);
            fieldCanvas.strokeLine(robotX, robotY, arrowX, arrowY);

            packet.put("X", robotX);
            packet.put("Y", robotY);
            packet.put("Theta (deg)", Math.toDegrees(robotTheta));
            packet.put("Left Elevator", Elevators.getLeftEncoder());
            packet.put("Right Elevator", Elevators.getRightEncoder());

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData("Left Elevator", Elevators.getLeftEncoder());
            telemetry.addData("Right Elevator", Elevators.getRightEncoder());
            telemetry.addData("Pivot Angle", Pivot.getAngle());
            telemetry.update();
        }
    }

}


