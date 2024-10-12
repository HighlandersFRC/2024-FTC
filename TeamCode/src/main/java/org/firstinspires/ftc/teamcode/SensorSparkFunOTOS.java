package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Commands.MoveToPosition;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.Strafe;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.LowPassFilter;
import org.json.JSONException;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@Autonomous(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {

    
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals.initialize(hardwareMap);
        LowPassFilter filterX = new LowPassFilter(1);
        CommandScheduler scheduler = new CommandScheduler();
        Drive.initialize(hardwareMap);
       Mouse.init(hardwareMap);
        

        boolean timing = false;
        waitForStart();


        scheduler.schedule(new SequentialCommandGroup(scheduler,new MoveToPosition(0.05,0,0)));

        while (opModeIsActive()) {
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
