package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.WristDefault;

public class Wrist extends Subsystem {
    private Servo wrist;
    private double position; // Default position


    public Wrist(String name, HardwareMap hardwareMap) {
        super(name);
        this.wrist = null;
        initialize(hardwareMap);
    }

    private void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        // Set initial position during initialization
    }

    public void contolWrist(Gamepad gamepad1) {

        if (gamepad1.dpad_right) {
           double wristPosition = 0;
            setPosition(wristPosition);
        } else if (gamepad1.dpad_left) {
            double wristPosition = 1;
            setPosition(wristPosition);
        } else if (gamepad1.dpad_up) {
            double wristPosition = 0.5;
            setPosition(wristPosition);
        }
    }

    public double getPosition() {
        return wrist.getPosition();
    }

    public void setPosition(double pos) {
        // Ensure position is within valid range (adjust min/max as needed)
        position = Math.max(0.0, Math.min(1.0, pos));
        wrist.setPosition(position);
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new WristDefault();
    }
}