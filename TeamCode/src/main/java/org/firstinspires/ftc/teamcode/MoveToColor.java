/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot systems
        Robot.initialize(hardwareMap);
        SparkFunOTOS mouse;
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update the robot's pose
            FinalPose.poseUpdate();

            // Retrieve gamepad inputs
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = gamepad1.left_stick_x;   // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset the yaw and encoders if the right bumper is pressed
            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            //debug stuff
            if (gamepad1.a) {
                Drive.drive(1,0,0,0);
            }
            if (gamepad1.b) {
                Drive.drive(0,1,0,0);
            }
            if (gamepad1.y){
                Drive.drive(0,0,1,0);
            }
            if (gamepad1.x){
                Drive.drive(0,0,0,1);
            }



            // Get robot's current heading
            double botHeading = Peripherals.getYaw();

            // Rotate gamepad inputs to align with the field
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Slightly boost the strafe power
            rotX *= 1.1;

            // Normalize motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Drive the motors
            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

            // Clean telemetry
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

            telemetry.addLine("Encoders")
                    .addData("Left Encoder", Drive.getLeftEncoder())
                    .addData("Right Encoder", Drive.getRightEncoder())
                    .addData("Center Encoder", Drive.getCenterEncoder());

            telemetry.addLine("Sensors")
                    .addData("Current Sensor State", FieldOfMerit.currentState);

            telemetry.update();


        }

    }

}*//*
*/
/*
*//*

*/
/*
*//*
*/
/*

*//*

*/
/*
*//*
*/
/*
*//*

*/
/*

*//*
*/
/*

*//*

*/
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    private Limelight3A limelight;

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

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double targetTx = Double.MAX_VALUE;

            if (result != null && result.isValid()) {
                double ty = result.getTy();
                double ta = result.getTa();

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double tx = colorTarget.getTargetXDegrees();
                    double area = colorTarget.getTargetArea();
                    telemetry.addData("Color Target", "Area: " + area + "%, tx: " + tx);

                    if (Math.abs(tx) < Math.abs(targetTx)) {
                        targetTx = tx;
                    }
                }

                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = Peripherals.getYaw();

            if (gamepad1.left_bumper) {
                if (targetTx != Double.MAX_VALUE) {
                    double turnSpeed = targetTx / 30; // Scale the turning speed
                    rx = targetTx > 0 ? -turnSpeed : turnSpeed;
                } else {
                    rx = 0;
                }
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
            telemetry.addLine("Encoders")
                    .addData("Left Encoder", Drive.getLeftEncoder())
                    .addData("Right Encoder", Drive.getRightEncoder())
                    .addData("Center Encoder", Drive.getCenterEncoder());
            telemetry.addLine("Sensors")
                    .addData("Current Sensor State", FieldOfMerit.currentState);
            telemetry.update();
        }
    }
}
*//*
*/
/*
*//*

*/
/*
*//*
*/
/*

*//*

*/
/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.initialize(hardwareMap);
        SparkFunOTOS mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(0);
        limelight.getStatus();
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double targetTx = Double.MAX_VALUE;

            if (result != null && result.isValid()) {
                double ty = result.getTy();
                double ta = result.getTa();

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double tx = colorTarget.getTargetXDegrees();
                    double area = colorTarget.getTargetArea();
                    telemetry.addData("Color Target", "Area: " + area + "%, tx: " + tx);

                    if (Math.abs(tx) < Math.abs(targetTx)) {
                        targetTx = tx;
                    }
                }

                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
                } else {
                    rx = 0;
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
}
*//*
*/
/*
*//*

*/
/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;
    private double Kp = -0.05;
    private double Ki = 0.0;
    private double Kd = -0.000002;

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

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        pid = new PID(Kp, Ki, Kd);
        pid.setMaxOutput(1.0);
        pid.setMinOutput(-1.0);
        pid.setSetPoint(0);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double targetTx = Double.MAX_VALUE;

            if (result != null && result.isValid()) {
                double ty = result.getTy();
                double ta = result.getTa();

                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double tx = colorTarget.getTargetXDegrees();
                    double area = colorTarget.getTargetArea();
                    telemetry.addData("Color Target", "Area: " + area + "%, tx: " + tx);

                    if (Math.abs(tx) < Math.abs(targetTx)) {
                        targetTx = tx;
                    }
                }

                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = Peripherals.getYaw();

            if (gamepad1.left_bumper) {
                if (targetTx != Double.MAX_VALUE) {
                    pid.setSetPoint(0);
                    double pidOutput = pid.updatePID(targetTx);
                    rx = pidOutput;
                } else {
                    rx = 0;
                }
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


            telemetry.addData("PID", Kp);
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
            telemetry.addLine("Encoders")
                    .addData("Left Encoder", Drive.getLeftEncoder())
                    .addData("Right Encoder", Drive.getRightEncoder())
                    .addData("Center Encoder", Drive.getCenterEncoder());
            telemetry.addLine("Sensors")
                    .addData("Current Sensor State", FieldOfMerit.currentState);
            telemetry.update();
        }
    }
}*//*
*/
/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;
    private double Kp = -0.05;
    private double Ki = 0.0;
    private double Kd = -0.000002;

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
                targetTy = result.getTy(); // Distance-related angle
                List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
                for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                    double tx = colorTarget.getTargetXDegrees();
                    if (Math.abs(tx) < Math.abs(targetTx)) {
                        targetTx = tx; // Closest x angle to center
                    }
                }
            }

            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotation

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
                    y = 0.3 * Math.cos(Math.toRadians(targetTy)); // Drive towards the target
                } else {
                    rx = 0;
                    y = 0;
                }
            } else {
                limelight.pipelineSwitch(0);
            }

            // Adjust for field-centric controls
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

            // Display telemetry data
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
}
*//*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class FieldCentric extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;
    private double Kp = -0.05;
    private double Ki = 0.0;
    private double Kd = -0.000002;

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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
}
*/
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class MoveToColor extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;
    private double Kp = -0.05;
    private double Ki = 0.0;
    private double Kd = -0.000002;

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

        pid = new PID(Kp, Ki, Kd);
        pid.setMaxOutput(1.0);
        pid.setMinOutput(-1.0);
        pid.setSetPoint(0);

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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = Peripherals.getYaw();

            if (gamepad1.left_bumper) {
                limelight.pipelineSwitch(1);
                if (targetTx != Double.MAX_VALUE) {
                    pid.setSetPoint(0);
                    double pidOutput = pid.updatePID(targetTx);
                    rx = pidOutput;
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
}
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.firstinspires.ftc.teamcode.Tools.PID;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp
public class MoveToColor extends LinearOpMode {
    private Limelight3A limelight;
    private PID pid;
    private double Kp = -0.05;
    private double Ki = 0.0;
    private double Kd = -0.000002;

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

        if (isStopRequested()) return null;

        pid = new PID(Kp, Ki, Kd);
        pid.setMaxOutput(1.0);
        pid.setMinOutput(-1.0);
        pid.setSetPoint(0);

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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                Peripherals.resetYaw();
                Drive.resetEncoder();
            }

            double botHeading = Peripherals.getYaw();

            if (gamepad1.left_bumper) {
                limelight.pipelineSwitch(1);
                if (targetTx != Double.MAX_VALUE) {
                    pid.setSetPoint(0);
                    double pidOutput = pid.updatePID(targetTx);
                    rx = pidOutput;
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
                    .addData("Theta (deg)", "%.2f", FinalPose.yaw);
            telemetry.update();
        }
        return null;
    }
}
