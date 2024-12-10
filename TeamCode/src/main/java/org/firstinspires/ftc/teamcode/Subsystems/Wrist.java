package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends Subsystem{
    static Servo wrist;

    public static void initialize(HardwareMap hardwareMap){

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setDirection(Servo.Direction.REVERSE);

    }
    public static void move(double position){
        wrist.setPosition(position);
    }
}