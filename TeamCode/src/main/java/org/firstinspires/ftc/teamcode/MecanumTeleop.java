package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.EncoderTicksToDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem("Intake", hardwareMap);
        Wrist wristSubsystem = new Wrist("Wrist", hardwareMap);
        Drive driveSubsystem = new Drive("Drive", hardwareMap);
        ElevatorSubsystem elevator = new ElevatorSubsystem("Elevator", hardwareMap);
        ArmSubsystem arm = new ArmSubsystem("Arm", hardwareMap);


        waitForStart();
        while (opModeIsActive()) {
            driveSubsystem.FeildCentric(gamepad1);
            elevator.manual(gamepad2);
            intakeSubsystem.controlIntake(gamepad1);
            arm.contolArm(gamepad2);

            wristSubsystem.setPosition(arm.wristPos);


        telemetry.addData("Arm Current Power", arm.pivot.getPower());
        telemetry.update();
        }
    }
}