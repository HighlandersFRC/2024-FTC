package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;


@Autonomous
public class DriveForwardAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        Mouse.configureOtos();
        Drive drive = new Drive("drive",hardwareMap);
        Superstructure superstructure = new Superstructure("superstructure");
        superstructure.init(hardwareMap);


        drive.setPosition(0.928, 2.821, 0);

        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "DriveForwardAuto1Meter.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        drive = new Drive("drive", hardwareMap);
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower moveToPosition;
        NewRobot robot = new NewRobot(hardwareMap);


        scheduler.setNewRobot(robot);

 /*       try {
            scheduler.schedule(new Wait(3000));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
 */
        waitForStart();
        try {
            //moveToPosition = new PolarPathFollower(drive, peripherals, pathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            //scheduler.schedule(moveToPosition);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }


        while (opModeIsActive()) {
            FinalPose.poseUpdate();


            superstructure.periodic();
            scheduler.run();




            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.Yaw;

            double robotDriveX = drive.getOdometryX();
            double robotDriveY = drive.getOdometryY();
            double robotDriveTheta = drive.getOdometryTheta();
            telemetry.addData("X", -robotY);
            telemetry.addData("Y", -robotX);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData("Drive X", robotDriveX);
            telemetry.addData("Drive Y", robotDriveY);
            telemetry.addData("Drive Theta", robotDriveTheta);
            telemetry.addData("I am ", " a Skibidi sigma");


            telemetry.update();
        }
    }
}
