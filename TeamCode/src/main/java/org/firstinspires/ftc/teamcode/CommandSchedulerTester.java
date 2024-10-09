package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.TestCommand;
import org.firstinspires.ftc.teamcode.Commands.TestCommand2;
import org.json.JSONException;

@TeleOp
public class CommandSchedulerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        CommandScheduler scheduler = new CommandScheduler();
        scheduler.schedule(new SequentialCommandGroup(scheduler, new TestCommand(), new TestCommand2()));
        while (opModeIsActive()) {
            telemetry.addData("Stick X", gamepad1.left_stick_x);
            telemetry.update();
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
