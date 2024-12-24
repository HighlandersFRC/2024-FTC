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
    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean armControlToggle = true;
        boolean togglePressed = false;
        boolean isManualControlActive = false;

        waitForStart();
        ArmSubsystem.initialize(hardwareMap);
        IntakeSubsystem.initialize(hardwareMap);
        Wrist.initialize(hardwareMap);
        Drive.initialize(hardwareMap);

        while (opModeIsActive()) {

            if (gamepad1.touchpad_finger_2 && !togglePressed) {
                armControlToggle = !armControlToggle;
                togglePressed = true;
            } else if (!gamepad1.touchpad_finger_2) {
                togglePressed = false;
            }

            if (armControlToggle) {
                // Manual control using left_bumper and right_bumper
                if (gamepad2.left_bumper) {
                    ArmSubsystem.setPower(1); // Set upward power for manual control
                    isManualControlActive = true;
                } else if (gamepad2.right_bumper) {
                    ArmSubsystem.setPower(-1); // Set downward power for manual control
                    isManualControlActive = true;
                } else {
                    if (isManualControlActive) {
                        // Capture the current position when manual control ends
                        pos = ArmSubsystem.getCurrentPositionWithLimitSwitch();
                        isManualControlActive = false;
                    }

                    // Automatic position control using buttons
                    if (gamepad2.y) {
                        pos = encodersToDeg(120);
                    } else if (gamepad2.x) {
                        pos = encodersToDeg(190);
                    } else if (gamepad2.dpad_down) {
                        pos = encodersToDeg(215);
                    } else if (gamepad2.b) {
                        pos = encodersToDeg(0);
                    }

                    // Apply PID control
                    power = 0.7;
                    piviotPID.setSetPoint(pos);
                    piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                    piviotPID.setMaxOutput(power);
                    piviotPID.setMinOutput(-power);
                    ArmSubsystem.setPower(-piviotPID.getResult());
                }
            } else {
                // Manual control using left_bumper and right_bumper
                if (gamepad1.left_bumper) {
                    ArmSubsystem.setPower(1); // Set upward power for manual control
                    isManualControlActive = true;
                } else if (gamepad1.right_bumper) {
                    ArmSubsystem.setPower(-1); // Set downward power for manual control
                    isManualControlActive = true;
                } else {
                    if (isManualControlActive) {
                        // Capture the current position when manual control ends
                        pos = ArmSubsystem.getCurrentPositionWithLimitSwitch();
                        isManualControlActive = false;
                    }

                    // Automatic position control using buttons
                    if (gamepad1.y) {
                        pos = encodersToDeg(120);
                    } else if (gamepad1.x) {
                        pos = encodersToDeg(190);
                    } else if (gamepad1.dpad_down) {
                        pos = encodersToDeg(215);
                    } else if (gamepad1.b) {
                        pos = encodersToDeg(0);
                    }

                    // Apply PID control
                    power = 0.7;
                    piviotPID.setSetPoint(pos);
                    piviotPID.updatePID(ArmSubsystem.getCurrentPositionWithLimitSwitch());
                    piviotPID.setMaxOutput(power);
                    piviotPID.setMinOutput(-power);
                    ArmSubsystem.setPower(-piviotPID.getResult());
                }
            }

            // Drive subsystem
            Drive.teleopDrive(gamepad1);
            Mouse.update();

            // Telemetry data
            telemetry.addData("Gamepad Toggle State", armControlToggle ? "Gamepad1" : "Gamepad2");
            telemetry.addData("Degrees for the Arm", getDegrees(ArmSubsystem.getCurrentPositionWithLimitSwitch()));
            telemetry.addData("Degrees for the Drive", getDegrees(Drive.leftBackPos()));
            telemetry.update();
        }
    }
}
