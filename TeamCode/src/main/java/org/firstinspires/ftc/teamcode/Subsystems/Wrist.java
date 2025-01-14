package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.ElevatorDefault;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.WristDefault;

public class Wrist extends Subsystem {
    static Servo wrist1;
    static Servo wrist2;

    public Wrist(String name) {
        super(name);
    }

    public static void initialize(HardwareMap hardwareMap) {

        wrist1 = hardwareMap.get(Servo.class, "wrist`");
        wrist2 = hardwareMap.get(Servo.class,"wrist2");

        wrist1.setDirection(Servo.Direction.REVERSE);


    }

    public static void move1(double position) {
        wrist1.setPosition(position);
    }
public static void move2(double position){
        wrist2.setPosition(position);
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
