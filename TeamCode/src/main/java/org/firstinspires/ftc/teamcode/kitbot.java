package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

@TeleOp
public class kitbot extends LinearOpMode {
    PID piviotPID = new PID(0.15, 0, 1);
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Drive.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        ArmSubsystem.initializeWithOutLimit(hardwareMap);

        while (opModeIsActive()) {

            Wrist.controlWrist(gamepad1);
            IntakeSubsystem.controlIntake(gamepad1);
            Drive.teleopDrive(gamepad1);
            ArmSubsystem.controlPivotWithoutLimitSwitch(gamepad1,pivotPID);

            telemetry.addData("Status", "Running");
            telemetry.addData("armPos",ArmSubsystem.getCurrentPosition());
            telemetry.update();
        }
    }
}
