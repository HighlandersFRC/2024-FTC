package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends Subsystem {


    private static Servo wristServo;

    private static final double Sample_Intake = 0.45;
    private static final double Specimen_Intake = 0.65;
    private static final double Place = 1.0;
    private static final double specimen_Place = 0.76;


    public static void initialize(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "wrist");


        wristServo.setPosition(Sample_Intake);
    }


    public static void setPositionSample(double position) {
        wristServo.setPosition(Sample_Intake);
    }


    public static void setPositionSpecimen() {
        wristServo.setPosition(Specimen_Intake);
    }

    public static void setPositionPlace() {
        wristServo.setPosition(Place);
    }

    public static void setPositionSpecimenPlace() {
        wristServo.setPosition(specimen_Place);
    }
}
