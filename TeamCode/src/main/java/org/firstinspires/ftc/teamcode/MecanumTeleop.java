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
public class MecanumTeleop extends LinearOpMode {
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

        double Pos = 0;

        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousDpadDown = false;

        ArmDefault armDefault = new ArmDefault(armSubsystem);
        waitForStart();
        armSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                ArmUp Up = new ArmUp(robot.arm);
                scheduler.schedule(Up);
            } else if (gamepad1.right_bumper) {
                ArmDown Down = new ArmDown(robot.arm);
                scheduler.schedule(Down);
            } else {
                StopArm Stop = new StopArm(robot.arm);
                scheduler.schedule(Stop);
            }

            scheduler.run();


        }
    }
}