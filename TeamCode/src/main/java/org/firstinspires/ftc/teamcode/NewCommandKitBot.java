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
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;


@TeleOp
public class NewCommandKitBot extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        NewArmSubsystem armSubsystem = new NewArmSubsystem("armSubsystem");
        NewElevatorSubsystem elevatorSubsystem = new NewElevatorSubsystem("elevatorSubsystem");
        NewWristSubsystem wristSubsystem = new NewWristSubsystem("wristSubsystem");
        NewIntakeSubsystem intakeSubsystem = new NewIntakeSubsystem("intakeSubsystem");
        Drive drive = new Drive("drive", hardwareMap);

        armSubsystem.init(hardwareMap);
        elevatorSubsystem.init(hardwareMap);
        wristSubsystem.init(hardwareMap);
        intakeSubsystem.init(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.arm = armSubsystem;
        robot.elevator = elevatorSubsystem;
        robot.wrist = wristSubsystem;
        robot.intake = intakeSubsystem;

        scheduler.setNewRobot(robot);
        waitForStart();

        while (opModeIsActive()) {
            elevatorSubsystem.periodic();
            armSubsystem.periodic();
            wristSubsystem.periodic();
            intakeSubsystem.periodic();


            if (gamepad1.b) {
                scheduler.schedule(new NewArmCommandDown(robot.arm));
            } else if (gamepad1.y) {
                scheduler.schedule(new NewArmCommandUp(robot.arm));
            } else if (gamepad1.x) {
                scheduler.schedule(new NewArmCommandSpecimen(robot.arm));
            } else if (gamepad1.a) {
                scheduler.schedule(new NewArmCommandHighBucket(robot.arm));
            }

            if (gamepad1.right_bumper) {
                scheduler.schedule(new NewElevatorCommandExtend(robot.elevator));
            } else if (gamepad1.left_bumper) {
                scheduler.schedule(new NewElevatorCommandRetract(robot.elevator));
            } else {
                scheduler.schedule(new NewElevatorCommandStop(robot.elevator));
            }

            if (gamepad1.dpad_up) {
                scheduler.schedule(new NewWristCommandUp(robot.wrist));
            } else if (gamepad1.dpad_down) {
                scheduler.schedule(new NewWristCommandDown(robot.wrist));
            }

            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandIntake(robot.intake));
            } else if(gamepad1.left_trigger > 0) {
                scheduler.schedule(new NewIntakeCommandOuttake(robot.intake));
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
