package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class kitbot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Drive.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        ArmSubsystem.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        while (opModeIsActive()) {

            Wrist.controlWrist(gamepad1);
            IntakeSubsystem.controlIntake(gamepad1);
            Drive.teleopDrive(gamepad1);
            ArmSubsystem.controlPivot(gamepad1,pivotPID);
            Mouse.update();

            telemetry.addData("Drive train back left", Drive.backLeftMotor.getCurrentPosition());
            telemetry.addData("Drive train front left", Drive.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Drive train back right", Drive.backRightMotor.getCurrentPosition());
            telemetry.addData("Drive train front right", Drive.frontRightMotor.getCurrentPosition());
            telemetry.addData("Mouse sensor X", Mouse.getX());
            telemetry.addData("Mouse sensor Y", Mouse.getY());
            telemetry.update();
        }
    }
}