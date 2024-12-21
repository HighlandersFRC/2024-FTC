package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ElevatorDefault;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.WristDefault;

public class Wrist extends Subsystem {
    static Servo wrist;

    public Wrist(String name) {
        super(name);
    }

    public static void initialize(HardwareMap hardwareMap) {

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setDirection(Servo.Direction.REVERSE);


    }

    public static void move(double position) {
        wrist.setPosition(position);
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(new WristDefault());
    }

    @Override
    public Command getDefaultCommand() {
        return new WristDefault();
    }
}
