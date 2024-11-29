package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

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
            Drive.FeildCentric(gamepad1);
            ArmSubsystem.controlPivot(gamepad1,piviotPID);
            Mouse.update();

            telemetry.addData("DriveCommand train back left", Drive.leftBackPos());
            telemetry.addData("DriveCommand train front left", Drive.leftFrontPos());
            telemetry.addData("DriveCommand train back right", Drive.RightBackPos());
            telemetry.addData("DriveCommand train front right", Drive.RightFrontPos());
            telemetry.addData("Piviot arm current position: ", ArmSubsystem.pivotMotor.getCurrentPosition());
            telemetry.addData("Mouse sensor X", Mouse.getX());
            telemetry.addData("Mouse sensor Y", Mouse.getY());
            telemetry.update();
        }
    }
}