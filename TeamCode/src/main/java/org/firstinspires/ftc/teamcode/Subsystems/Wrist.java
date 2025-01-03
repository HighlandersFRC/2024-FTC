package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.WristDefault;

public class Wrist extends Subsystem {
    private Servo wrist;
    private double position = 0.4; // Default position
ArmSubsystem armSubsystem;

    public Wrist(String name, HardwareMap hardwareMap) {
        super(name);
        this.wrist = null;
        initialize(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        // Set initial position during initialization
        wrist.setPosition(position);
    }

    public void controlWrist(Gamepad gamepad1) {
        if (gamepad1.dpad_up) {
            position = 0.4; // Adjust positions as needed
        } else if (gamepad1.dpad_left) {
            position = 0.8;
        } else if (gamepad1.dpad_right) {
            position = 0.0;
        }
if (armSubsystem.wristPosition == 0.35) {
    setPosition(position);
} else {
    setPosition(armSubsystem.wristPosition);
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