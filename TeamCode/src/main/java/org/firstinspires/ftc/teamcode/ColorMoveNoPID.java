/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
*/
/*import org.firstinspires.ftc.teamcode.Tools.FinalPose;*//*

import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class ColorMoveNoPID extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;


    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initialize(hardwareMap);
        SparkFunOTOS mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(1);
        limelight.getStatus();
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double targetTx = Double.MAX_VALUE;
            double targetTy = Double.MAX_VALUE;

            if (result != null && result.isValid()) {
                targetTy = result.getTy();
                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double tx = colorTarget.getTargetXDegrees();
                    if (Math.abs(tx) < Math.abs(targetTx)) {
                        targetTx = tx;
                    }
                }
            }

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = Peripherals.getYaw();

            if (gamepad1.left_bumper) {
                limelight.pipelineSwitch(1);
                if (targetTx != Double.MAX_VALUE) {
                    double turnSpeed = targetTx / 30;
                    rx = targetTx > 0 ? -turnSpeed : turnSpeed;
                    y = 0.3 * Math.cos(Math.toRadians(targetTy));
                } else {
                    rx = 0;
                    y = 0;
                }
            } else {
                limelight.pipelineSwitch(0);
            }

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());
            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());
            telemetry.addLine("Mouse Sensor Values")
                    .addData("X (m)", "%.2f", Drive.getTotalXTraveled())
                    .addData("Y (m)", "%.2f", Drive.getTotalYTraveled())
                    .addData("Theta (deg)", "%.2f", Drive.totalThetaTraveled);
            telemetry.addLine("Fused Pose")
                    .addData("X (m)", "%.2f", FinalPose.x)
                    .addData("Y (m)", "%.2f", FinalPose.y)
                    .addData("Theta (deg)", "%.2f", FinalPose.Yaw);
            telemetry.update();
        }
    }
}*/
