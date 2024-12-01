
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeSubsystem extends Subsystem {
    public static CRServo intake;

    public IntakeSubsystem(String name) {
        super();
    }

    public static void initialize(HardwareMap hardwareMap) {
        intake = hardwareMap.crservo.get("intake");
    }

    public static void setPower(double power) {
        intake.setPower(power);
    }

    // Add this method to handle gamepad inputs for intake control
    public static void controlIntake(Gamepad gamepad1) {

        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
    }

    public static void contolIntakeWithDriverOperator(Gamepad gamepad2, Gamepad gamepad1) {
        if (gamepad2.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad2.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
        if (gamepad1.left_trigger != 0) {
            IntakeSubsystem.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            IntakeSubsystem.setPower(-gamepad1.right_trigger);
        } else {
            IntakeSubsystem.setPower(0);
        }
    }
}

