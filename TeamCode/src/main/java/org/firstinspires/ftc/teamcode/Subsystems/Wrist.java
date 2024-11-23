package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends Subsystem {
    public static CRServo wrist;

    public static void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.crservo.get("wrist");
    }

    public static void setPower(double power) {
        wrist.setPower(power);
    }

    // Add this method to handle gamepad inputs for wrist control
    public static void controlWrist (Gamepad gamepad1) {
        if (gamepad1.left_bumper) {
            Wrist.setPower(1);
        } else if (gamepad1.right_bumper) {
            Wrist.setPower(-1);
        } else {
            Wrist.setPower(0);
        }
    }
}
