package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.LowPassFilter;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;


@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {
    SparkFunOTOS mouse;


    @Override
    public void runOpMode() throws InterruptedException {
        Drive.initialize(hardwareMap);
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        LowPassFilter filterX = new LowPassFilter(0.0001);


        configureOtos();
        boolean timing = false;
        waitForStart();

        double startTimeA = 0;
        double elapsedTime = 0;
        double currentPosX = 0;
        double lastPosX = 0;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;

            double rx = -gamepad1.right_stick_x;
            SparkFunOTOS.Pose2D pos = mouse.getPosition();
            double sensorReadingX = pos.x;  // Replace with your sensor input
            double filteredReading = filterX.filter(sensorReadingX);  // Apply the filter

            currentPosX = sensorReadingX;
            double PosDifX = currentPosX - lastPosX;
            lastPosX = sensorReadingX;

            if (gamepad1.y) {
                mouse.resetTracking();
            }

            if (gamepad1.x) {
                mouse.calibrateImu();
            }

            if (PosDifX != 0 ) {
                startTimeA = System.currentTimeMillis();


            } else if (PosDifX == 0.0) {
                System.out.println("DifX1"+PosDifX);
                System.out.println(PosDifX==0.0);
                elapsedTime = (System.currentTimeMillis() - startTimeA) / 1000.0;


            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = y / denominator;
            double backLeftPower = y / denominator;
            double frontRightPower = y / denominator;
            double backRightPower = y / denominator;
            Drive.drive(-frontLeftPower, -frontRightPower, -backLeftPower, -backRightPower);
            double realXVel = PosDifX / elapsedTime;

            System.out.println("time"+timing);
            System.out.println("time"+elapsedTime);
            System.out.println("DifX"+pos.x);
            System.out.println("DifX_Working"+filteredReading);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("Time Elapsed (seconds)", elapsedTime);
            telemetry.addData("X coordinate (filtered)", realXVel);
            telemetry.addData("PosDifX", PosDifX);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        mouse.setOffset(offset);
        mouse.setLinearScalar(1.27485976543);
        mouse.setAngularScalar(1);
        mouse.calibrateImu();
        mouse.resetTracking();
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
