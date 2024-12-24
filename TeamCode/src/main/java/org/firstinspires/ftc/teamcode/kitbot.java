package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.encodersToDeg;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.piviotPID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class kitbot extends LinearOpMode {
    public static double pos;
    public void runOpMode() throws InterruptedException {
        boolean armControlToggle = true;
        boolean togglePressed = false;
        waitForStart();
        ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        while (opModeIsActive()) {

            if (gamepad1.left_stick_button && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.left_stick_button) {
                togglePressed = false;
            }

            if (armControlToggle) {
             if (!gamepad2.y || !gamepad2.x || !gamepad2.dpad_down || !gamepad2.b) {

                 pos = ArmSubsystem.getCurrentPositionWithLimitSwitch();
             }
                 // Use PID for preset positions
                 if (gamepad2.y) {
                     pos = encodersToDeg(120);
                     telemetry.addData("Control Mode", "PID (Preset Y)");
                 } else if (gamepad2.x) {
                     pos = encodersToDeg(190);
                     telemetry.addData("Control Mode", "PID (Preset X)");
                 } else if (gamepad2.dpad_down) {
                     pos = encodersToDeg(215);
                     telemetry.addData("Control Mode", "PID (Preset Down)");
                 } else if (gamepad2.b) {
                     pos = encodersToDeg(0);
                     telemetry.addData("Control Mode", "PID (Preset B)");
                 }

                 // Update PID
                 piviotPID.setSetPoint(pos);
                 piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                 piviotPID.setMaxOutput(0.7);
                 piviotPID.setMinOutput(-0.7);
                 ArmSubsystem.setPower(-piviotPID.getResult());

                // Handle manual movement
                ArmSubsystem.ArmMovement(gamepad2);
                IntakeSubsystem.contolIntakeBlueAlliance(gamepad2);
                Wrist.controlWrist(gamepad2);

            } else {
                // Gamepad2 control (unchanged)
                ArmSubsystem.ArmMovement(gamepad1);
                IntakeSubsystem.contolIntakeBlueAlliance(gamepad1);
                Wrist.controlWrist(gamepad1);

                if (!gamepad1.right_bumper || !gamepad1.left_bumper) {
                    if (gamepad1.y) {
                        pos = encodersToDeg(120);
                    } else if (gamepad1.x) {
                        pos = encodersToDeg(190);
                    } else if (gamepad1.dpad_down) {
                        pos = encodersToDeg(215);
                    } else if (gamepad1.b) {
                        pos = encodersToDeg(0);
                    }

                    piviotPID.setSetPoint(pos);
                    piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                    piviotPID.setMaxOutput(0.7);
                    piviotPID.setMinOutput(-0.7);
                    ArmSubsystem.setPower(-piviotPID.getResult());
                }

                gamepad2.rumble(1);
            }


            Drive.FeildCentric(gamepad1);
            Mouse.update();

            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad1" : "Gamepad2");
            telemetry.addData("Degrees", getDegrees(ArmSubsystem.getCurrentPositionWithLimitSwitch()));

            telemetry.update();
        }
    }
}
