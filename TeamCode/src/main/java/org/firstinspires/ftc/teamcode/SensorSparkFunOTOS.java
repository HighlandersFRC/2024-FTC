package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.Tools.KalmanFilter;

@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {
    SparkFunOTOS mouse;
    KalmanFilter kalmanFilterX; // Kalman Filter for X coordinate
    KalmanFilter kalmanFilterY; // Kalman Filter for Y coordinate

    @Override
    public void runOpMode() throws InterruptedException {
        Drive.initialize(hardwareMap);
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        // Initialize Kalman Filters with appropriate dimensions and noise covariances
        int stateDim = 2; // Assuming 2D position tracking
        double[][] processNoiseCov = {{1e-4, 0}, {0, 1e-4}}; // Tune this
        double[][] measurementNoiseCov = {{1e-2, 0}, {0, 1e-2}}; // Tune this
        double[][] initialCovariance = {{1, 0}, {0, 1}};
        double[] initialStateX = {0, 0}; // Initial state for X
        double[] initialStateY = {0, 0}; // Initial state for Y

        kalmanFilterX = new KalmanFilter(stateDim, processNoiseCov, measurementNoiseCov, initialCovariance, initialStateX);
        kalmanFilterY = new KalmanFilter(stateDim, processNoiseCov, measurementNoiseCov, initialCovariance, initialStateY);

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

            // Update Kalman Filters with the new measurements
            kalmanFilterX.predict(state -> state); // Predict step (simple identity for now)
            kalmanFilterX.update(new double[]{pos.x}, state -> state); // Measurement update for X

            kalmanFilterY.predict(state -> state); // Predict step (simple identity for now)
            kalmanFilterY.update(new double[]{pos.y}, state -> state); // Measurement update for Y

            // Get the filtered positions
            double filteredPosX = kalmanFilterX.currentState()[0];
            double filteredPosY = kalmanFilterY.currentState()[0];

            // Calculate PosDifX with the filtered X position
            currentPosX = filteredPosX;
            double PosDifX = currentPosX - lastPosX;
            lastPosX = currentPosX;

            if (gamepad1.y) {
                mouse.resetTracking();
            }

            if (gamepad1.x) {
                mouse.calibrateImu();
            }

            if (PosDifX != 0 && !timing) {
                startTimeA = System.currentTimeMillis();
                timing = true;
                sleep(1);
            } else if (PosDifX == 0 && timing) {
                elapsedTime = (System.currentTimeMillis() - startTimeA) / 1000.0;
                timing = false;
                sleep(1);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = y / denominator;
            double backLeftPower = y / denominator;
            double frontRightPower = y / denominator;
            double backRightPower = y / denominator;
            Drive.drive(-frontLeftPower, -frontRightPower, -backLeftPower, -backRightPower);
            double realX = filteredPosX / elapsedTime;

            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();
            telemetry.addData("Timing", timing);
            telemetry.addData("Time Elapsed (seconds)", elapsedTime);
            telemetry.addData("X coordinate (filtered)", filteredPosX);
            telemetry.addData("Y coordinate (filtered)", filteredPosY);
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
