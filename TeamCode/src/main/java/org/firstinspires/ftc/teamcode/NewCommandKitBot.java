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
        NewArmSubsystem armSubsystem = new NewArmSubsystem("arm");
        NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("elevator");
        NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("intake");
        NewWristSubsystem wristSubsystem = new NewWristSubsystem("wrist");
        Drive drive = new Drive("drive", hardwareMap);

        armSubsystem.init(hardwareMap);
        elevatorSubsystem.init(hardwareMap);
        intakeSubsystem.init(hardwareMap);
        wristSubsystem.init(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);

        scheduler.setNewRobot(robot);
        waitForStart();

        while (opModeIsActive()) {
            armSubsystem.periodic();
            elevatorSubsystem.periodic();
            intakeSubsystem.periodic();
            wristSubsystem.periodic();


            if (gamepad1.b) {
                scheduler.schedule(new NewArmCommandDown(robot.armSubsystem));
            } else if (gamepad1.y) {
                scheduler.schedule(new NewArmCommandUp(robot.armSubsystem));
            } else if (gamepad1.x) {
                scheduler.schedule(new NewArmCommandSpecimen(robot.armSubsystem));
            } else if (gamepad1.a) {
                scheduler.schedule(new NewArmCommandHighBucket(robot.armSubsystem));
            }

            if (gamepad1.right_bumper) {
                scheduler.schedule(new NewElevatorCommandExtend(robot.elevatorSubsystem));
            } else if (gamepad1.left_bumper) {
                scheduler.schedule(new NewElevatorCommandRetract(robot.elevatorSubsystem));
            } else {
                scheduler.schedule(new NewElevatorCommandStop(robot.elevatorSubsystem));
            }

            if (gamepad1.dpad_up) {
                scheduler.schedule(new NewWristCommandUp(robot.wristSubsystem));
            } else if (gamepad1.dpad_down) {
                scheduler.schedule(new NewWristCommandDown(robot.wristSubsystem));
            }

            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandIntake(robot.intakeSubsystem));
            } else if(gamepad1.left_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandOuttake(robot.intakeSubsystem));
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
