package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends LinearOpMode {
    private CommandScheduler scheduler;

    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = new CommandScheduler();

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new ParallelCommandGroup(scheduler, Parameters.ANY, new Wait(10000), new Wait(1000000000)),
                new TestCommand());

        scheduler.schedule(commandGroup);


        while (!isStopRequested()) {
            while (opModeIsActive()){
                scheduler.run();
            }
            {
                commandGroup.end();
            }
            sleep(50);
        }
    }
}
