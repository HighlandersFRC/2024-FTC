package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@TeleOp
public class IntakeControlOpMode extends LinearOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakeSubsystem.intakeSample();
            }

            if (gamepad1.b) {
                intakeSubsystem.intake();
            }

            if (gamepad1.x) {
                intakeSubsystem.outtake();
            }

            if (gamepad1.y) {
                intakeSubsystem.stopIntake();
            }

            telemetry.addData("Detected Color", intakeSubsystem.getDetectedColor());
            telemetry.update();
        }
    }
}
