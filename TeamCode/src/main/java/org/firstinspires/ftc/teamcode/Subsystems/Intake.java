
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Intake {

    private static NormalizedColorSensor colorSensor;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private static final String setColor = "red";



    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");

        Intake.stopIntake();
    }

    public void intake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public void outtake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public static void stopIntake() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public boolean getCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green && red > 0.01) {
            mainColor = "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            mainColor = "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");
    }
}