package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Robot.elevators;
import static org.firstinspires.ftc.teamcode.Tools.Robot.intake;
import static org.firstinspires.ftc.teamcode.Tools.Robot.pivot;
import static org.firstinspires.ftc.teamcode.Tools.Robot.wrist;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Elevator;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.PivotMove;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Commands.WristMove;
import org.firstinspires.ftc.teamcode.Subsystems.Elevators;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;

@TeleOp
public class Drive extends LinearOpMode {

    private boolean wristPositionToggled = false;
    private double toggledWristPosition = 0.1 - Constants.WRIST_OFFSET;
    private double defaultWristPosition = 0.8 - Constants.WRIST_OFFSET;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        Robot.CURRENT_STATE = "Tele-Op";
        Elevators.resetEncoders();

        while (opModeIsActive()) {
            Mouse.update();
            FinalPose.poseUpdate();

            if (gamepad2.right_bumper) {
                scheduler.schedule(new PivotMove(pivot, 99));
            } else if (gamepad2.left_bumper) {
                scheduler.schedule(new PivotMove(pivot, -10));
            } else if (gamepad2.b) {
                scheduler.schedule(new PivotMove(pivot, 0));
            }

            if (gamepad2.dpad_right) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new Elevator(elevators, 0), new Wait(50), new PivotMove(pivot, -10)));
            }

            if (gamepad2.x) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new PivotMove(pivot, 90), new Wait(50), new Elevator(elevators, 1000)));
            }

            if (gamepad1.left_stick_button) {
                wristPositionToggled = !wristPositionToggled;
                telemetry.addData("Wrist Toggle State", wristPositionToggled);
                telemetry.update();
            }

            if (wristPositionToggled) {
                scheduler.schedule(new WristMove(wrist, toggledWristPosition));
            } else {
                if (Elevators.getLeftEncoder() < 200 && Pivot.getAngle() > 90) {
                    scheduler.schedule(new WristMove(wrist, defaultWristPosition));
                } else if (pivot.getAngle() < 90) {
                    scheduler.schedule(new WristMove(wrist, 0.8 - Constants.WRIST_OFFSET));
                } else {
                    scheduler.schedule(new WristMove(wrist, 0.6 - Constants.WRIST_OFFSET));
                }
            }

            if (gamepad1.right_trigger > 0.1) {
                scheduler.schedule(new IntakeCommand(intake));
            }

            if (gamepad1.left_trigger > 0.1) {
                Intake.leftServo.setPower(1);
                Intake.rightServo.setPower(-1);
            }

            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                org.firstinspires.ftc.teamcode.Subsystems.Drive.RobotCentric(leftStickX / 4, leftStickY / 4, rightStickX / 4);
            } else {
                org.firstinspires.ftc.teamcode.Subsystems.Drive.RobotCentric(leftStickX, leftStickY, rightStickX);
            }

            if (gamepad1.right_bumper) {
                org.firstinspires.ftc.teamcode.Subsystems.Drive.stop();
            }

            if (gamepad2.dpad_left) {
                Elevators.resetEncoders();
            }
            if (gamepad1.options) {
                Pivot.resetEncoder();
            }
            if (gamepad1.dpad_left) {
                scheduler.schedule(new PivotMove(pivot, -100));
            } else if (gamepad1.dpad_right) {
                scheduler.schedule(new PivotMove(pivot, 100));
            }

            scheduler.run();

            telemetry.addLine("Pivot").addData("Encoder", Pivot.getEncoderPosition()).addData("Pivot Angle", Pivot.getAngle()).addData("Limit Switch", Pivot.limitSwitch.getState());
            telemetry.addLine("Elevator").addData("Left Encoder", Elevators.getLeftEncoder()).addData("Right Encoder", Elevators.getRightEncoder());
            telemetry.addLine("Pose").addData("x", FinalPose.x).addData("y", FinalPose.y).addData("current", FieldOfMerit.currentState).addData("yaw", FinalPose.yaw);
            telemetry.update();

            Robot.elevatorPower = (((Math.sqrt(gamepad2.right_trigger)) / Math.pow(gamepad2.right_trigger - 2, 2))) - gamepad2.left_trigger;

            scheduler.printCurrentCommands();
        }
    }
}
