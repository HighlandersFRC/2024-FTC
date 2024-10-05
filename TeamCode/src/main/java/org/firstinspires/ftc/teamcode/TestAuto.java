package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.MoveToPosition;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.json.JSONException;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);

        Drive.setPosition(0, 0, 0);

        CommandScheduler scheduler = new CommandScheduler();
        Command moveToPosition = new SequentialCommandGroup(scheduler, new MoveToPosition(0.5, 0, 0), new MoveToPosition(0.5, 0.5, 0), new MoveToPosition(0.0, 0.5, 0), new MoveToPosition(0,0,0));

        waitForStart();

        try {
            scheduler.schedule(moveToPosition);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {
            FinalPose.poseUpdate();
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

            telemetry.addData("X", FinalPose.x);
            telemetry.addData("Y", FinalPose.y);
            telemetry.addData("Theta", Peripherals.getYawDegrees());
            telemetry.update();

        }
    }
}
