package org.firstinspires.ftc.teamcode;

import android.content.res.Resources;
import android.text.style.WrapTogetherSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Elevator;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PivotMove;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Commands.WristMove;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

import java.util.Scanner;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Pivot.initialize(hardwareMap);
        Intake.initialize(hardwareMap);
        Elevators.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();

        while (opModeIsActive()){
            Mouse.update();
            FinalPose.poseUpdate();

            if (gamepad1.y){
                scheduler.schedule(new PivotMove(94));
            }
            else if (gamepad1.a){
                scheduler.schedule(new PivotMove(-10));
            }else if (gamepad1.b){
                scheduler.schedule(new PivotMove(0));
            }else {
                if (Pivot.getAngle() < 20 && PivotMove.pivotPID.getSetPoint() < 0){
                    Pivot.setPower(0);
                    Pivot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }else {
                    Pivot.setPower(PivotMove.pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));

                }
            }


            if (gamepad1.dpad_right) {
                Robot.CURRENT_ELEVATOR = 0;
                scheduler.schedule(new SequentialCommandGroup(scheduler, new Elevator(), new Wait(350), new PivotMove(-10)));
            }

            if (gamepad1.x) {
                Robot.CURRENT_ELEVATOR = 1000;
                scheduler.schedule(new SequentialCommandGroup(scheduler, new PivotMove(90), new Wait(350), new Elevator()));
            }

            if (gamepad1.dpad_down){
                scheduler.schedule(new WristMove(0.2));
            }
            if (gamepad1.dpad_up){
                scheduler.schedule(new WristMove(1));
            }
          /*  if (Pivot.getAngle() > 10){
            if (gamepad1.left_bumper){
                Robot.CURRENT_ELEVATOR = Elevators.getLeftEncoder() - 200;
                scheduler.schedule(new Elevator());
            }else if (gamepad1.right_bumper){
                Robot.CURRENT_ELEVATOR = Elevators.getLeftEncoder() + 200;
                scheduler.schedule(new Elevator());
            }}else{*/
                if (gamepad1.right_bumper){
                    Robot.CURRENT_ELEVATOR = Elevators.getLeftEncoder();
                    Elevators.moveLeftElevator(1);
                    Elevators.moveRightElevator(1);
                }
                else if (gamepad1.left_bumper){
                    Robot.CURRENT_ELEVATOR = Elevators.getLeftEncoder();
                    Elevators.moveRightElevator(-1);
                    Elevators.moveLeftElevator(-1);
                }else {
                    Robot.CURRENT_ELEVATOR = Elevators.getLeftEncoder();
                    Elevators.moveLeftElevator(0);
                    Elevators.moveRightElevator(0);
                    Elevators.setBrakeMode();
                }


            if (gamepad1.right_trigger > 0.1) {

                scheduler.schedule(new IntakeCommand(hardwareMap));
            }
            if (gamepad1.left_trigger > 0.1) {

                Intake.leftServo.setPower(1);
                Intake.rightServo.setPower(-1);
/*
                commandScheduler.schedule(new IntakeCommand(hardwareMap, OUTTAKE));
*/
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = 0;
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            org.firstinspires.ftc.teamcode.Subsystems.Drive.drive(frontLeftPower, frontRightPower, -backLeftPower, backRightPower);

                scheduler.run();
            telemetry.addLine("Pivot")
                    .addData("Encoder", Pivot.getEncoderPosition())
                    .addData("Pivot Angle", Pivot.getAngle());


            telemetry.addLine("Elevator")
                    .addData("Left Encoder", Elevators.getLeftEncoder())
                    .addData("Right Encoder", Elevators.getRightEncoder());
            telemetry.addLine("Pose")
                    .addData("x", FinalPose.x)
                    .addData("y", FinalPose.y)
                    .addData("current", FieldOfMerit.currentState)
                    .addData("yaw", FinalPose.yaw);
            telemetry.update();

        }
    }
}
