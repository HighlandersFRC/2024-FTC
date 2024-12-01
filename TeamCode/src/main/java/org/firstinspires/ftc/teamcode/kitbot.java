package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class kitbot extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
    waitForStart();
    ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        while (opModeIsActive()) {
            ArmSubsystem.controlPivot(gamepad1,piviotPID);
            IntakeSubsystem.controlIntake(gamepad1);
            Wrist.controlWrist(gamepad1);
            Drive.FeildCentric(gamepad1);
            ArmSubsystem.gamepad1Climb(gamepad1, piviotPID);
            Mouse.update();
        }
    }
}