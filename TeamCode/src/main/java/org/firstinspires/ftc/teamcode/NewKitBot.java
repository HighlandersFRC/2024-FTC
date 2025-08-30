package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.NewArmSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.NewElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.NewWristSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

@TeleOp
public class NewKitBot extends LinearOpMode {

Superstructure superstructure = new Superstructure("superstructure");

    @Override
    public void runOpMode() throws InterruptedException {

      Drive drive = new Drive("Drive",hardwareMap);
      superstructure.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
           superstructure.periodic();

           if (gamepad2.a) {

           }




drive.FeildCentric(gamepad1);
        }
    }
}
