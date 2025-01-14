
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
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

    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();
        Robot robot = new Robot(hardwareMap);
        Drive drive = new Drive("drive",hardwareMap);
        drive.setPosition(0, 0, 0);

        PathLoading pathLoading = new PathLoading(hardwareMap.appContext, "Speicaman.polarpath");
        PathLoading path2 = new PathLoading(hardwareMap.appContext, ".polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        drive = new Drive("drive", hardwareMap);
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower moveToPosition;

 /*       try {
            scheduler.schedule(new Wait(3000));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
 */
        waitForStart();
        try {
            scheduler.schedule(new SequentialCommandGroup(scheduler,  new PolarPathFollower(drive, peripherals, PathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler),new WristCommands(robot.wrist, 0.75), new ArmCommand(robot.arm, DegreesToEncoderTicks(150))));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }


        while (opModeIsActive()) {
            FinalPose.poseUpdate();
            scheduler.run();

            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.Yaw;
            telemetry.addData("X", -robotY);
            telemetry.addData("Y", -robotX);
            telemetry.addData("Theta", robotTheta);


            telemetry.update();
        }
    }
}
