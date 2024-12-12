package org.firstinspires.ftc.teamcode;

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
import org.json.JSONException;

@TeleOp
public class CommandKitBot extends LinearOpMode {

    public static double armPos;
public static double pos;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                armPos = 0;
            } else if (gamepad1.x) {
                armPos= -1964;
            } else if (gamepad1.dpad_down) {
                armPos = -3365;
          } else if (gamepad1.a){
                armPos = -3000;
            }
            if (gamepad1.dpad_down || gamepad1.b || gamepad1.x || gamepad1.a) {
                scheduler.overrideSpecificCommand(new ArmCommand(armPos),ArmCommand.class);
            }


            if (gamepad1.right_trigger != 0) {
                StopIntake.StopTheIntake = false;
                scheduler.overrideSpecificCommand(new Intake(), Outtake.class);
            } else if (gamepad1.left_trigger != 0) {
                StopIntake.StopTheIntake = false;
                scheduler.overrideSpecificCommand(new Outtake(), Intake.class);
            } else {
                StopIntake.StopTheIntake = true;
            }
            if (gamepad1.dpad_up) {
                pos = 0.4;
        }else if(gamepad1.dpad_right) {
                pos = 0.8;
            } else if(gamepad1.dpad_left) {
                pos = 0;
            }
            if(gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                scheduler.schedule(new WristCommands(pos));
            }

            Drive.FeildCentric(gamepad1);

            try {
                scheduler.removeDuplicateCommands();
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
telemetry.addData("Mouse Sensor Y:", Mouse.getX());
            telemetry.addData("Mouse Sensor X:",   Mouse.getY());
telemetry.addData("Piviot Arm Posiotion:", ArmSubsystem.getCurrentPositionWithLimitSwitch());
            telemetry.update();





        }
    }
}