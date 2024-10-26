package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Vision;

public class Robot {
    public static void initialize(HardwareMap hardwareMap) {

        Peripherals.initialize(hardwareMap);
        Drive.initialize(hardwareMap);
        FieldOfMerit.initialize(hardwareMap);
        Vision.initialize(hardwareMap);
    }
}
