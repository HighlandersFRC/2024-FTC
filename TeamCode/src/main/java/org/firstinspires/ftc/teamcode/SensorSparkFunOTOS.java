
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.json.JSONException;

@Autonomous(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {


    @Override

    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive("drive",hardwareMap);
        Peripherals.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);


        boolean timing = false;

        waitForStart();
        FinalPose.Reset();


        while (opModeIsActive()) {

            Mouse.update();
            FinalPose.poseUpdate();

            scheduler.run();

            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("X", Mouse.getX());
            telemetry.addData("Y", Mouse.getY());
            telemetry.addData("Heading angle",Mouse.getTheta());
            telemetry.update();
        }
    }


}
