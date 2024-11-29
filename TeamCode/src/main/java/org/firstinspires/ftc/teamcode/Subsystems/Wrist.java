
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends Subsystem {
    public static Servo wrist;
public static double position;
    public static void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
    }

    // Add this method to handle gamepad inputs for wrist control
    public static void controlWrist (Gamepad gamepad1) {

if (gamepad1.left_bumper && gamepad1.right_bumper) {
    position = 0.5;
    } else if (gamepad1.left_bumper) {
            position = 0.8;
        } else if (gamepad1.right_bumper) {
           position = 0.2;
        }



        wrist.setPosition(position);
    }

    public static double getPosition() {
        return wrist.getPosition();
    }

}
