package org.firstinspires.ftc.teamcode;

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

        Peripherals.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        

        boolean timing = false;

        waitForStart();
        Mouse.configureOtos();

        try {
            scheduler.schedule(new SequentialCommandGroup(scheduler,new org.firstinspires.ftc.teamcode.Commands.Drive(hardwareMap,0.5,0.5)));
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {

            Mouse.update();
            FinalPose.poseUpdate();

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("X", Mouse.getX()/0.83766124979);
            telemetry.addData("Y", Mouse.getY());
            telemetry.addData("Heading angle",Mouse.getTheta());
            telemetry.update();
        }
    }


}
