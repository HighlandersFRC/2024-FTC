package org.firstinspires.ftc.teamcode.Tools;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class Mouse {
    SparkFunOTOS mouse;


    public void runOpMode()  {
        Drive.initialize(hardwareMap);
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        configureOtos();
        boolean timing = false;


        double startTimeA = 0;
        double elapsedTime = 0;
        double currentPosX = 0;
        double lastPosX = 0;


            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            SparkFunOTOS.Pose2D pos = mouse.getPosition();
            currentPosX = pos.x;
            double PosDifX = currentPosX - lastPosX;
            lastPosX = pos.x;

            if (gamepad1.y) {
                mouse.resetTracking();
            }

            if (gamepad1.x) {
                mouse.calibrateImu();
            }

            if (PosDifX != 0 && !timing) {
                // Start timing when PosDifX is first non-zero
                startTimeA = System.currentTimeMillis();
                timing = true;
            } else if (PosDifX == 0 && timing) {
                // Stop timing when PosDifX returns to zero
                elapsedTime = (System.currentTimeMillis() - startTimeA) / 1000.0; // Convert to seconds
                timing = false;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = y / denominator;
            double backLeftPower = y / denominator;
            double frontRightPower = y / denominator;
            double backRightPower = y / denominator;
            Drive.drive(-frontLeftPower, -frontRightPower, -backLeftPower, -backRightPower);
            double realX = pos.x / elapsedTime;  // Assuming elapsedTimeA is the time to calculate velocity

            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("Time Elapsed (seconds)", elapsedTime);
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("PosDifX", PosDifX);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();

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

