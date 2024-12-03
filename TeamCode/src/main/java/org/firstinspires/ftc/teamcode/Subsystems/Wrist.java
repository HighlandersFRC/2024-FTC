
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends Subsystem {
    public static Servo wrist;
public static double position = 0.49;
    public static void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
    }

    // Add this method to handle gamepad inputs for wrist control
    public static void controlWrist (Gamepad gamepad1) {

if (gamepad1.dpad_left) {
    position = 0.49;
    } else if (gamepad1.dpad_up) {
            position = 0.8;
        } else if (gamepad1.dpad_down) {
           position = 0.2;
        }



        wrist.setPosition(position);
    }
public static void contolWristWithOperator(Gamepad gamepad2) {
    if (gamepad2.left_bumper && gamepad2.right_bumper) {
        position = 0.49;
    } else if (gamepad2.right_bumper) {
        position = 0.8;
    } else if (gamepad2.left_bumper) {
        position = 0.2;
    }
}
    public static double getPosition() {
        return wrist.getPosition();
    }

}
