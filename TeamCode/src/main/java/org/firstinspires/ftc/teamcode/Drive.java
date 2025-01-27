package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Robot.elevatorPower;
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

    private boolean wristPositionToggled = false; // Tracks if wrist position has been toggled
    private double defaultWristPosition = 0.8; // Default wrist position when pivot angle is high

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot.initialize(hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();

        Robot.CURRENT_STATE = "Tele-Op";

        while (opModeIsActive()) {
            gamepad2.rumble(1000);
            gamepad1.rumble(100);

            Mouse.update();
            FinalPose.poseUpdate();

            // Control for Pivot
            if (gamepad2.right_bumper) {
                scheduler.schedule(new PivotMove(pivot, 99));
            } else if (gamepad2.left_bumper) {
                scheduler.schedule(new PivotMove(pivot, -10));
            } else if (gamepad2.b) {
                scheduler.schedule(new PivotMove(pivot, 0));
            }

            // Elevator and Wrist Control
            if (gamepad2.dpad_right) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new Elevator(elevators, 0), new Wait(50), new PivotMove(pivot, -10)));
            }

            if (gamepad2.x) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new PivotMove(pivot, 90), new Wait(50), new Elevator(elevators, 1000)));
            }

            // Control for Wrist (toggle on press of the left stick button on gamepad1)
            if (gamepad1.left_stick_button) {
                wristPositionToggled = !wristPositionToggled; // Toggle wrist position on button press
            }

            // Set wrist position based on pivot angle and toggle
            if (wristPositionToggled) {
                scheduler.schedule(new WristMove(wrist, 0.1)); // Set to position 0.1 if toggled
            } else {
                if (pivot.getAngle() < 90) {
                    scheduler.schedule(new WristMove(wrist, 0.8)); // Low pivot angle, wrist at 0.1
                } else {
                    scheduler.schedule(new WristMove(wrist, 0.6)); // High pivot angle, wrist at 0.8
                }
            }

            // Intake Control


            if (gamepad1.right_trigger > 0.1) {
                scheduler.schedule(new IntakeCommand(intake));
            }

            if (gamepad1.left_trigger > 0.1) {

                Intake.leftServo.setPower(1);
                Intake.rightServo.setPower(-1);

            }

            // Driving Control
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

            if (gamepad2.dpad_left){
                Elevators.resetEncoders();
            }
            if (gamepad1.options){
                Pivot.resetEncoder();
            }
            if (gamepad1.dpad_left){
                Pivot.setPower(-0.3);
            }else
            if (gamepad1.dpad_right){
                Pivot.setPower(0.3);
            }else {
                Pivot.setPower(0);
            }


            // Run the scheduler to execute any pending commands
            scheduler.run();

            // Telemetry Data
            telemetry.addLine("Pivot").addData("Encoder", Pivot.getEncoderPosition()).addData("Pivot Angle", Pivot.getAngle());
            telemetry.addLine("Elevator").addData("Left Encoder", Elevators.getLeftEncoder()).addData("Right Encoder", Elevators.getRightEncoder());
            telemetry.addLine("Pose").addData("x", FinalPose.x).addData("y", FinalPose.y).addData("current", FieldOfMerit.currentState).addData("yaw", FinalPose.yaw);
            telemetry.update();

            // Update the elevator power for gamepad2 input
            Robot.elevatorPower = (((Math.sqrt(gamepad2.right_trigger)) / Math.pow(gamepad2.right_trigger - 2, 2))) - gamepad2.left_trigger;

            scheduler.printCurrentCommands();
        }
    }
}
