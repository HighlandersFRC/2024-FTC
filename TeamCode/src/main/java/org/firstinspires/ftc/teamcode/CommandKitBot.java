package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.ArmCommand;
import org.firstinspires.ftc.teamcode.Commands.ArmDown;
import org.firstinspires.ftc.teamcode.Commands.ArmUp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ArmDefault;
import org.firstinspires.ftc.teamcode.Commands.ElevatorDown;
import org.firstinspires.ftc.teamcode.Commands.ElevatorUp;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.StopArm;
import org.firstinspires.ftc.teamcode.Commands.StopElevator;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

@TeleOp
public class CommandKitBot extends LinearOpMode {
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        dashboard = FtcDashboard.getInstance();
        ArmSubsystem armSubsystem = new ArmSubsystem("arm", hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem("intakeSubsystem", hardwareMap);
        Wrist wrist = new Wrist("wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);

        Robot robot = new Robot(hardwareMap);
        robot.arm = armSubsystem;
        robot.drive = driveSubsystem;
        robot.intakeSubsystem = intake;
        robot.wrist = wrist;
        scheduler.setRobot(robot);


        ArmDefault armDefault = new ArmDefault(armSubsystem);
        waitForStart();
        armSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new ArmCommand(robot.arm, DegreesToEncoderTicks(35)), new WristCommands(robot.wrist, 0)));
            } else if (gamepad1.b) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new WristCommands(robot.wrist, 0.6), new ArmCommand(robot.arm, DegreesToEncoderTicks(0))));
            } else if (gamepad1.x) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new ArmCommand(robot.arm, DegreesToEncoderTicks(90)), new WristCommands(robot.wrist, 0)));
            } else if (gamepad1.y) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new ArmCommand(robot.arm, DegreesToEncoderTicks(120)), new WristCommands(robot.wrist, 0)));
            }


            if (gamepad1.right_trigger != 0) {
                scheduler.schedule(new Intake(robot.intakeSubsystem));
            } else if (gamepad1.left_trigger != 0) {
                scheduler.schedule(new Outtake(robot.intakeSubsystem));
            }

            if (gamepad1.right_bumper) {
                scheduler.schedule(new ElevatorUp(robot.elevator));
            } else if (gamepad1.left_bumper) {
                scheduler.schedule(new ElevatorDown(robot.elevator));
            } else {
                scheduler.schedule(new StopElevator(robot.elevator));
            }


            scheduler.removeDuplicateCommands();
            scheduler.run();
            scheduler.printCurrentCommands();

        }
    }
}