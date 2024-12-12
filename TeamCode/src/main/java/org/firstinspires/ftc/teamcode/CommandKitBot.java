package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Commands.ArmCommandDown;
import org.firstinspires.ftc.teamcode.Commands.ArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.StopIntake;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.json.JSONException;

@TeleOp
public class CommandKitBot extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                scheduler.overrideSpecificCommand(new ArmCommandUp(-1964), ArmCommandDown.class);
            } else if (gamepad1.b) {
                scheduler.overrideSpecificCommand(new ArmCommandDown(10), ArmCommandUp.class);
            } else if (gamepad1.x) {
                scheduler.overrideSpecificCommand(new ArmCommandUp(-3000), ArmCommandDown.class);
            } else if (gamepad1.dpad_down) {
                scheduler.overrideSpecificCommand(new ArmCommandUp(-3365), ArmCommandDown.class);
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
            double pos = 0.4;
            if (gamepad1.dpad_up) {
                pos = 0.4;
        }else if(gamepad1.dpad_right) {
                pos = 0.8;
            } else if(gamepad1.dpad_left) {
                pos = 0;
            }
           scheduler.overrideSpecificCommand(new WristCommands(pos), WristCommands.class);
            ArmSubsystem.gamepad1Climb(gamepad1);
            Drive.FeildCentric(gamepad1);

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }






        }
    }
}