package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensorSubsystem extends Subsystem{
    public static NormalizedColorSensor colorSensor;

    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }
 public static void colorSensor() {
//            if(blue > red && blue > green) {
//                IntakeSubsystem.intake.setPower(1);
//            } else {
//                IntakeSubsystem.intake.setPower(0);
//            }
 }

    public static double getRed(){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        return color.red;
    }

    public static double getGreen() {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        return color.green;
    }

    public static double getBlue() {
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        return color.blue;
    }
}
