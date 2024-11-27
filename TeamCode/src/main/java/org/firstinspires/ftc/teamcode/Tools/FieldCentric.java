package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo;
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Elevator.initialize(hardwareMap);
        Pivot.initialize(hardwareMap);
        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {




           FinalPose.poseUpdate();

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

           if(gamepad1.a){
               Pivot.runUsingPID(700);
           }

           if(gamepad1.b){
               Pivot.runUsingPID(30);
           }
           Pivot.run();


           if(gamepad1.x){
               Elevator.runUsingPID(800,800);
           }
           if(gamepad1.right_bumper){
               Elevator.extendUp();
           }else if(gamepad1.left_bumper){
               Elevator.extendDown();
           }else{
               Elevator.stop();
           }
           Elevator.run();

          if(gamepad1.right_trigger > 0.1){
              IntakeSubsystem.intakeSample();
          }else if(gamepad1.left_trigger>0.1){
              IntakeSubsystem.outtake();
          }else{
              IntakeSubsystem.stopIntake();
          }
          if(gamepad1.y){
              IntakeSubsystem.intake();
          }



          if(gamepad1.dpad_up){
              Wrist.setPositionPlace();
          } else if (gamepad1.dpad_right) {
              Wrist.setPositionSpecimen();
          }else if(gamepad1.dpad_down){
              Wrist.setPositionPlace();
          }else if (gamepad1.dpad_left){
              Wrist.setPositionSpecimenPlace();
          }


            double botHeading = -Mouse.getTheta();

           /* double botHeading = 0;*/

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            Drive.drive(-frontLeftPower, -frontRightPower, backLeftPower, -backRightPower);



            telemetry.addLine("pivot Encoder")
                            .addData("pivot",Pivot.getEncoderPosition());
            telemetry.addLine("Elevators")
                            .addData("left",Elevator.getLeftEncoderPosition())
                            .addData("right", Elevator.getRightEncoderPosition());
            telemetry.addLine("IMU Data")
                    .addData("Yaw (Degrees)", "%.2f", Peripherals.getYawDegrees());

            telemetry.addLine("Odometry")
                    .addData("X (m)", "%.2f", Drive.getOdometryX())
                    .addData("Y (m)", "%.2f", Drive.getOdometryY())
                    .addData("Theta (deg)", "%.2f", Drive.getOdometryTheta());

            telemetry.addLine("Mouse Sensor Values")
                    .addData("X (m)", "%.2f", Mouse.getX())
                    .addData("Y (m)", "%.2f", Mouse.getY())
                    .addData("Theta (deg)", "%.2f", Mouse.getTheta());
            telemetry.addLine("Limelight Values")
                            .addData("x", Peripherals.getLimelightX())
                                    .addData("y", Peripherals.getLimelightY());
            telemetry.addLine("Fused Pose")
                    .addData("X (m)", "%.2f", FinalPose.x)
                    .addData("Y (m)", "%.2f", FinalPose.y)
                    .addData("Theta (deg)", "%.2f", FinalPose.yaw);

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
