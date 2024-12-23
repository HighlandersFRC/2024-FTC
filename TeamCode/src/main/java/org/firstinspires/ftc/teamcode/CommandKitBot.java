package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.Commands.ArmCommand;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.StopIntake;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import static org.firstinspires.ftc.teamcode.Commands.StopIntake.StopTheIntake;

import org.firstinspires.ftc.teamcode.Tools.Robot;
import org.json.JSONException;

@TeleOp
public class CommandKitBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();

        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        ArmCommand Score = new ArmCommand(-1964);
        ArmCommand Zero= new ArmCommand(0);

        scheduler = new CommandScheduler();

        Intake intakeCommand = new Intake(Robot.intakeSubsystem);
        Outtake outtakeCommand = new Outtake(Robot.intakeSubsystem);

        WristCommands leftWrist = new WristCommands(Robot.wrist,0.4);
        WristCommands rightWrist = new WristCommands(Robot.wrist,0.8);
        WristCommands zeroWrist = new WristCommands(Robot.wrist,0);


        while(opModeIsActive()) {
            if (gamepad1.right_trigger != 0) {
                StopTheIntake = false;
                scheduler.schedule(intakeCommand);
            } else if (gamepad1.left_trigger != 0) {
                StopTheIntake = false;
                scheduler.schedule(outtakeCommand);
            } else {
                StopTheIntake = true;
            }

            if (gamepad1.y) {


                scheduler.schedule(Score);
            } else if (gamepad1.b) {

                scheduler.schedule(Zero);
            }

            Drive.FeildCentric(gamepad1);
            scheduler.run();

            telemetry.addData("Mouse Sensor Y:", Mouse.getX());
            telemetry.addData("Mouse Sensor X:", Mouse.getY());
            telemetry.addData("Piviot Arm Posiotion:", ArmSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.update();
        }




    }
}
