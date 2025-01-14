package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp
public class TestClass extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem("Elevator", hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem("Intake", hardwareMap);
        Wrist wrist = new Wrist("wrist", hardwareMap);
        Drive drive = new Drive("drive", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
                elevatorSubsystem.manual(gamepad1);
                intake.controlIntake(gamepad1);
                double wristPos = 0.5;
                 if (gamepad1.dpad_right) {
                     wristPos = 1;
                 } else if (gamepad1.dpad_up) {
                  wristPos = 0.5;
                 } else if (gamepad1.dpad_left) {
                     wristPos = 0;
                 }
                 wrist.setPosition(wristPos);
                 drive.sketchDrive(gamepad1);
            telemetry.update();
        }
    }
}
