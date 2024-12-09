package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Commands.ArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Intake;
import org.firstinspires.ftc.teamcode.Commands.Outtake;
import org.firstinspires.ftc.teamcode.Commands.WristCommands;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.json.JSONException;

@TeleOp
public class CommandKitBot extends LinearOpMode {

    private boolean StopTheIntake;

    @Override
    public void runOpMode() throws InterruptedException {
double toto = 7;
        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.y) {
                scheduler.schedule(new ArmCommandUp(-1936));
            } else if(gamepad1.b) {
                scheduler.schedule(new ArmCommandUp(0));
            }


            if(gamepad1.right_trigger!=0) {
                    StopTheIntake=false;
                    scheduler.schedule(new Intake());
                } else if(gamepad1.left_trigger!=0) {
                    StopTheIntake=false;
                    scheduler.schedule(new Outtake());
                }
                  else {
                      StopTheIntake=true;
                  }

                  if(gamepad1.dpad_right) {
                      scheduler.schedule(new WristCommands(0.8));
                  } else if(gamepad1.dpad_left) {
                      scheduler.schedule(new WristCommands(0.2));
                  }

            Drive.FeildCentric(gamepad1);

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }




        }
    }
}