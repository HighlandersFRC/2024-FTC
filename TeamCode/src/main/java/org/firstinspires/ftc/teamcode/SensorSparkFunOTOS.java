// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {
    SparkFunOTOS mouse;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive.initialize(hardwareMap);
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        configureOtos();

        waitForStart();

        double startTime = 0;
        double elapsedTime = 0;
        double power = 0;
        boolean timing = false;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            SparkFunOTOS.Pose2D pos = mouse.getPosition();

            if (gamepad1.y) {
                mouse.resetTracking();
            }

            if (gamepad1.x) {
                mouse.calibrateImu();
            }
            if (gamepad1.a){
                power = 0.1;
            }
            else {
                power = 0;
            }



            if (gamepad1.a && !timing) {
                // Start timing when the button is first pressed
                startTime = System.currentTimeMillis();
                timing = true;

            } else if (!gamepad1.a && timing) {
                // Stop timing when the button is released
                elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0; // Convert to seconds
                timing = false;

            }


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (power) / denominator;
            double backLeftPower = (power) / denominator;
            double frontRightPower = (power) / denominator;
            double backRightPower = (power) / denominator;
            Drive.drive(-frontLeftPower, -frontRightPower, -backLeftPower, -backRightPower);

            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("timing",timing);
            telemetry.addData("X coordinate", pos.x * 1.27485976543);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.addData("Elapsed Time", timing ? (System.currentTimeMillis() - startTime) / 1000.0 : elapsedTime);
            telemetry.addData("power",Drive.backLeftMotor.getPower());
            telemetry.addData("true Elapsed time",elapsedTime);
            telemetry.addData("velocity x",   pos.x/elapsedTime);
            telemetry.addData("velocity y",   pos.y/elapsedTime);
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
