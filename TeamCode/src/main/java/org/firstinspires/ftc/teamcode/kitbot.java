package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.absoluteArmZero;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class kitbot extends LinearOpMode {
    public boolean armControlToggle = true;
    public boolean togglePressed = false;
    @Override
    public void runOpMode() throws InterruptedException {

        ArmSubsystem armSubsystem = new ArmSubsystem("Arm", hardwareMap);

        Wrist wristSubsystem = new Wrist("Wrist");
        Drive driveSubsystem = new Drive("Drive", hardwareMap, telemetry);


        waitForStart();



        wristSubsystem.initialize(hardwareMap);
        driveSubsystem.initialize(hardwareMap);

        while (opModeIsActive()) {

            if (gamepad1.touchpad && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.touchpad) {
                togglePressed = false;
            }

            Mouse.update();
            driveSubsystem.FeildCentric(gamepad1);
            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad2" : "Gamepad1");
            telemetry.addData("Wrist Pos", wristSubsystem.getPosition());
            telemetry.update();
        }
    }
}