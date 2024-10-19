package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.MoveToPosition;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.json.JSONException;

@Autonomous
public class TestAuto extends LinearOpMode {

/*
    private FtcDashboard dashboard;
*/

    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);

/*
        dashboard = FtcDashboard.getInstance();
*/

        Drive.setPosition(0, 0, 0);
        //just add the autonomous path here in the pathfilename parameter.  the path has to be in assets
        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Straight.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive();
        Peripherals peripherals = new Peripherals("peripherals");
        Command moveToPosition;
        try {
            moveToPosition = new PolarPathFollower(drive, peripherals, PathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        try {
            scheduler.schedule(moveToPosition);
        } catch (JSONException e) {
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
            double robotTheta = Peripherals.getYawDegrees();

  /*          TelemetryPacket packet = new TelemetryPacket();

            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = Peripherals.getYawDegrees();

           packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect(robotX - 9, robotY - 9, 18, 18)
                    .setStroke("black")
                    .strokeRect(robotX - 9, robotY - 9, 18, 18);

            double headingLength = 18;
            double headingX = robotX + headingLength * Math.cos(Math.toRadians(robotTheta));
            double headingY = robotY + headingLength * Math.sin(Math.toRadians(robotTheta));
            packet.fieldOverlay()
                    .setStroke("red")
                    .strokeLine(robotX, robotY, headingX, headingY);

            dashboard.sendTelemetryPacket(packet);
*/
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.update();
        }
    }
}
