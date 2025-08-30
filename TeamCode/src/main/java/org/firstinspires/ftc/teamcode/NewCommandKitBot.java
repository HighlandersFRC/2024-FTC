package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandDown;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandHighBucket;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandSpecimen;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.NewElevatorCommandExtend;
import org.firstinspires.ftc.teamcode.Commands.NewElevatorCommandRetract;
import org.firstinspires.ftc.teamcode.Commands.NewElevatorCommandStop;
import org.firstinspires.ftc.teamcode.Commands.NewIntakeCommandIntake;
import org.firstinspires.ftc.teamcode.Commands.NewIntakeCommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.NewWristCommandDown;
import org.firstinspires.ftc.teamcode.Commands.NewWristCommandUp;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;


@TeleOp
public class NewCommandKitBot extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Superstructure superstructure = new Superstructure("superstructure");
        Drive drive = new Drive("drive", hardwareMap);

        superstructure.init(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.superstructure = superstructure;
        scheduler.setNewRobot(robot);
        waitForStart();

        while (opModeIsActive()) {
            superstructure.periodic();


            if (gamepad1.b) {
                scheduler.schedule(new NewArmCommandDown(robot.superstructure));
            } else if (gamepad1.y) {
                scheduler.schedule(new NewArmCommandUp(robot.superstructure));
            } else if (gamepad1.x) {
                scheduler.schedule(new NewArmCommandSpecimen(robot.superstructure));
            } else if (gamepad1.a) {
                scheduler.schedule(new NewArmCommandHighBucket(robot.superstructure));
            }

            if (gamepad1.right_bumper) {
                scheduler.schedule(new NewElevatorCommandExtend(robot.superstructure));
            } else if (gamepad1.left_bumper) {
                scheduler.schedule(new NewElevatorCommandRetract(robot.superstructure));
            } else {
                scheduler.schedule(new NewElevatorCommandStop(robot.superstructure));
            }

            if (gamepad1.dpad_up) {
                scheduler.schedule(new NewWristCommandUp(robot.superstructure));
            } else if (gamepad1.dpad_down) {
                scheduler.schedule(new NewWristCommandDown(robot.superstructure));
            }

            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandIntake(robot.superstructure));
            } else if(gamepad1.left_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandOuttake(robot.superstructure));
            }

            drive.FeildCentric(gamepad1);
            scheduler.run();
            scheduler.printCurrentCommands();

            telemetry.addData("mouse X", Mouse.getX());
            telemetry.addData("mouse Y", Mouse.getY());
            telemetry.addData("mouse theta", Mouse.getTheta());
            telemetry.update();
        }
    }
}
