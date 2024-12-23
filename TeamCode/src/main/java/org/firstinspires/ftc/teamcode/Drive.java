package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Robot.elevatorPower;
import static org.firstinspires.ftc.teamcode.Tools.Robot.elevators;
import static org.firstinspires.ftc.teamcode.Tools.Robot.intake;
import static org.firstinspires.ftc.teamcode.Tools.Robot.pivot;
import static org.firstinspires.ftc.teamcode.Tools.Robot.wrist;

import android.content.res.Resources;
import android.text.style.WrapTogetherSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Elevator;
import org.firstinspires.ftc.teamcode.Commands.ElevatorWithPower;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
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

        CommandScheduler scheduler = new CommandScheduler();




        while (opModeIsActive()) {
            Mouse.update();
            FinalPose.poseUpdate();
       /*     if (gamepad1.a) {
                Pivot.setPower(-1);
            } else if (gamepad1.start) {
                Pivot.resetEncoder();
            }*/
            if (gamepad2.y) {
                scheduler.schedule(new PivotMove(pivot, 100));
            } else if (gamepad2.a) {
                scheduler.schedule(new PivotMove(pivot, -10));
            } else if (gamepad2.b) {
                scheduler.schedule(new PivotMove(pivot, 0));
            }/*else {
                if (Pivot.getAngle() < 20 && PivotMove.pivotPID.getSetPoint() < 0){
                    Pivot.setPower(0);
                    Pivot.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }else {
                    Pivot.setPower(PivotMove.pivotPower + (Constants.PIVOT_FEED_FORWARD * Math.cos(Math.toRadians(Pivot.getAngle()) + Constants.ARM_BALANCE_OFFSET)));

                }
            }*/

/*            if (gamepad1.left_bumper) {
                Pivot.setPower(1);
            } else if (gamepad1.right_bumper) {
                Pivot.setPower(-1);
            }*/
            if (gamepad2.dpad_right) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new Elevator(elevators, 0), new Wait(50), new PivotMove(pivot, -10)));
            }

            if (gamepad2.x) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new PivotMove(pivot, 90), new Wait(50), new Elevator(elevators, 1000)));
            }

            if (gamepad1.dpad_down) {
                scheduler.schedule(new WristMove(wrist, 0.1));
            }

            if (gamepad1.dpad_up) {
                scheduler.schedule(new WristMove(wrist, 0.8));
            }


            if (gamepad1.right_trigger > 0.1) {

                scheduler.schedule(new IntakeCommand(intake));
            }
            if (gamepad1.left_trigger > 0.1) {

                Intake.leftServo.setPower(1);
                Intake.rightServo.setPower(-1);

            }

            org.firstinspires.ftc.teamcode.Subsystems.Drive.RobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            scheduler.run();
            telemetry.addLine("Pivot").addData("Encoder", Pivot.getEncoderPosition()).addData("Pivot Angle", Pivot.getAngle());
            telemetry.addLine("Elevator").addData("Left Encoder", Elevators.getLeftEncoder()).addData("Right Encoder", Elevators.getRightEncoder());
            telemetry.addLine("Pose").addData("x", FinalPose.x).addData("y", FinalPose.y).addData("current", FieldOfMerit.currentState).addData("yaw", FinalPose.yaw);
            telemetry.update();

            Robot.elevatorPower = Robot.elevatorPowerCalc(gamepad2.right_bumper, gamepad2.left_bumper);

            scheduler.printCurrentCommands();
        }
    }
}
