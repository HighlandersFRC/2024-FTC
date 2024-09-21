package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.LowPassFilter;

@Autonomous(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {
    SparkFunOTOS mouse;

    @Override
    public void runOpMode() throws InterruptedException {

        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        LowPassFilter filterX = new LowPassFilter(1);
        CommandScheduler scheduler = new CommandScheduler();

        mouse.resetTracking();
        mouse.calibrateImu();

        configureOtos();
        boolean timing = false;
        waitForStart();

        double currentPosX = 0;
        double lastPosX = 0;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = mouse.getPosition();


            currentPosX = pos.x;
            double PosDifX = currentPosX - lastPosX;
            lastPosX = pos.x;


            System.out.println("time"+timing);
            System.out.println("X"+pos.x);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("X", pos.x);
            //telemetry.addData("Y", pos.y*1.365708773626061);
            telemetry.addData("diff x",PosDifX);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();
            scheduler.schedule(new SequentialCommandGroup(new Drive(hardwareMap,0.3,1)));
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        mouse.setAngularUnit(AngleUnit.DEGREES);
        mouse.setLinearUnit(DistanceUnit.METER );
        // distance from center of robot
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setOffset(offset);
        mouse.setLinearScalar(1);
        mouse.setAngularScalar(1);
        mouse.calibrateImu();
        mouse.resetTracking();
        // used for camara

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        mouse.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
