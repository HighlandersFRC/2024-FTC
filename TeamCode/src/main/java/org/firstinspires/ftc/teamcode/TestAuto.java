package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Parameters.ALL;
import static org.firstinspires.ftc.teamcode.Tools.Parameters.ANY;
import static org.firstinspires.ftc.teamcode.Tools.Parameters.SPECIFIC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends LinearOpMode {
    private CommandScheduler scheduler;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        scheduler = new CommandScheduler();

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelCommandGroup(scheduler, SPECIFIC, new Wait(10000), new Wait(1000)),
                new TestCommand());

        scheduler.schedule(commandGroup);

        while (!isStopRequested()) {
            if (!opModeInInit()) {
                while (opModeIsActive()) {
                    scheduler.run();
                }
            }
            sleep(50);
        }
    }
}