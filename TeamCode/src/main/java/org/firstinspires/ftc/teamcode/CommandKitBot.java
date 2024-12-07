package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.ArmCommandUp;
import org.firstinspires.ftc.teamcode.Commands.ArmCommmandDown;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.json.JSONException;

@TeleOp
public class CommandKitBot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
double toto = 7;
        ArmSubsystem.initialize(hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        while (opModeIsActive()) {

if(gamepad1.b) {
    scheduler.schedule(new ArmCommmandDown());
} else if(gamepad1.y) {
    scheduler.schedule(new ArmCommandUp());
}

                  //                  if(gamepad1.right_trigger!=0) {
//                    StopTheIntake=false;
//                    scheduler.schedule(new Intake());
//                } else if(gamepad1.left_trigger!=0) {
//                    StopTheIntake=false;
//                    scheduler.schedule(new Outtake());
//                }
//                  else {
//                      StopTheIntake=true;
//                  }

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            telemetry.addData("Current Arm Pos", ArmSubsystem.getCurrentPosition());

        }
    }
}