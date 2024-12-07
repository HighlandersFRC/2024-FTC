package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.json.JSONException;

@Autonomous
public class kitbot_auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drive.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        ArmSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Mouse.init(hardwareMap);


        CommandScheduler scheduler = new CommandScheduler();


        scheduler.schedule(new DriveCommand(hardwareMap,1,1));
        waitForStart();

        while (opModeIsActive()) {
            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

        }
    }
}
