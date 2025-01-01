
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.WristDefault;

public class Wrist extends Subsystem {
    public Servo wrist;
public double position = 0.4;
    public void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
    }

    public Wrist(String name) {
        super(name);
    }

    // Add this method to handle gamepad inputs for wrist control
    public void controlWrist (Gamepad gamepad1) {

if (gamepad1.dpad_up) {
    position = 0.4;
    } else if (gamepad1.dpad_left) {
            position = 0.8;
        } else if (gamepad1.dpad_right) {
           position = 0;
        }



        wrist.setPosition(position);
    }
public void contolWristWithOperator(Gamepad gamepad2) {
    if (gamepad2.left_bumper && gamepad2.right_bumper) {
        position = 0.49;
    } else if (gamepad2.right_bumper) {
        position = 0.8;
    } else if (gamepad2.left_bumper) {
        position = 0.2;
    }
}
    public double getPosition() {
        return wrist.getPosition();
    }

    public void setPosition(double pos){
        wrist.setPosition(pos



        );
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new WristDefault());
    }


    @Override
    public Command getDefaultCommand() {
        return new WristDefault(); // Retrieve the set default command
    }

}