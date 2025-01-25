package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.Commands.ElevatorUp;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.PathingTool.Path2;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.json.JSONException;

@Autonomous
public class TestAuto extends LinearOpMode {

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();
        Robot robot = new Robot(hardwareMap);
        Drive drive = new Drive("drive", hardwareMap);
        drive.setPosition(0, 0, 0);

        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Speciman.polarpath");
        Path2 path2 = new Path2(hardwareMap.appContext, "BackUps.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        drive = new Drive("drive", hardwareMap);
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower moveToPosition;

        waitForStart();
        try {
            scheduler.schedule(new SequentialCommandGroup(
                    scheduler,
                    new Wait(2000),
                    new PolarPathFollower(drive, peripherals, pathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler),
                    new Wait(2000),
                    new ArmCommand(robot.arm, DegreesToEncoderTicks(65)),
                    new Wait(2000),
                    new WristCommands(robot.wrist, 0.3),
                    new Wait(2000),
                    new ElevatorCommand(robot.elevator, -1700),
                    new Wait(2000),
                    new ElevatorCommand(robot.elevator, 0),
                    new Wait(2000),
                    new ArmCommand(robot.arm, DegreesToEncoderTicks(100)),
//                    new PolarPathFollower(drive, peripherals, path2.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler),
                    new ArmCommand(robot.arm, DegreesToEncoderTicks(0))
            ));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {
            FinalPose.poseUpdate();

            scheduler.run();


            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.Yaw;
            double currentTime = System.currentTimeMillis();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Robot X", -robotY);
            packet.put("Robot Y", -robotX);
            packet.put("Robot Theta", robotTheta);
            packet.put("Time", currentTime);
            packet.put("Target Angle", 150);
            packet.put("Current Pos", getDegrees(robot.arm.getCurrentPositionWithLimitSwitch()));
            packet.put("Result", piviotPID.getResult());
            packet.put("Arm current power", robot.arm.getPower());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", -robotY);
            telemetry.addData("Y", -robotX);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData("Time", currentTime);
            telemetry.update();
        }
    }
}