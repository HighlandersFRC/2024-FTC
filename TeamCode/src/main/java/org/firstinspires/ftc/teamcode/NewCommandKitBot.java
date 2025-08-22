package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandDown;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandHighBucket;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandSpecimen;
import org.firstinspires.ftc.teamcode.Commands.NewArmCommandUp;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;

public class NewCommandKitBot extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        NewArmSubsystem armSubsystem = new NewArmSubsystem("armSubsystem", gamepad1);
        armSubsystem.init(hardwareMap);

        NewArmCommandDown down = new NewArmCommandDown(armSubsystem);
        NewArmCommandUp up = new NewArmCommandUp(armSubsystem);
        NewArmCommandSpecimen specimen = new NewArmCommandSpecimen(armSubsystem);
        NewArmCommandHighBucket highBucket = new NewArmCommandHighBucket(armSubsystem);

        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                scheduler.schedule(down);
            } else if (gamepad1.b) {
                scheduler.schedule(up);
            } else if (gamepad1.x) {
                scheduler.schedule(specimen);
            } else if (gamepad1.y) {
                scheduler.schedule(highBucket);
            }

        }
    }
}
