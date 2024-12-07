package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

public class Robot {
    public static double CURRENT_PIVOT;
    public static double CURRENT_WRIST;
    public static double CURRENT_INTAKE_POWER;
    public static double CURRENT_ELEVATOR;
    public static void initialize(HardwareMap hardwareMap) {
        Peripherals.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        FieldOfMerit.initialize(hardwareMap);
        Vision.initialize(hardwareMap);
    }
    public static void run(){

    }
}
