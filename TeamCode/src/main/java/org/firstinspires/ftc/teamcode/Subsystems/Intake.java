/*

package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;


public class Intake extends Subsystem {

    public static NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo ;
    public static CRServo rightServo;
    private static final String setColor = "red";
    private boolean intakeStarted = false;
    private static boolean isStopped = false;

    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }


    }

    public static void intake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);

    }

    public static void outtake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public static String getDetectedColor() {
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

        return mainColor;
    }

    public boolean isCorrectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        String mainColor = "";
        if (red > blue && red > green && red >                                                                                                                                                        0.01) {
            mainColor = "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            mainColor = "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            mainColor = "yellow";
        }

        return mainColor.equals(setColor) || mainColor.equals("yellow");
    }

    public static void intakeSample() {



        if(!getDetectedColor().equals(setColor) && !getDetectedColor().equals("yellow")) {

            intake();
        }

        stopIntake();
    }

    public static void stopIntake() {

        leftServo.setPower(0);
        rightServo.setPower(0);
    }
}*/
/*

*//*

*/
/*
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Intake extends Subsystem {

    private static NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private static final String setColor = "red";
    private static boolean isIntaking = false;

    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public static void intake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);

    }

    public static void outtake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public static String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        if (red > blue && red > green && red > 0.01) {
            return "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            return "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            return "yellow";
        }
        return "";
    }

    public static boolean isCorrectColor() {
        String detectedColor = getDetectedColor();
        return detectedColor.equals(setColor) || detectedColor.equals("yellow");
    }

    public static void startIntakeSample() {
        isIntaking = true;
        intake();
    }

    public static void updateIntakeSample() {
        if (isIntaking) {
            String detectedColor = getDetectedColor();
            if (detectedColor.equals(setColor) || detectedColor.equals("yellow")) {
                stopIntake();
            }
        }
    }

    public static void stopIntake() {
        isIntaking = false;
        leftServo.setPower(0);

        rightServo.setPower(0);
    }
}
*//*
*/
/*

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Intake extends Subsystem {

    private static NormalizedColorSensor colorSensor = null;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private static final String setColor = "red";
    private static boolean isIntaking = false;

    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public static void intake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public static void outtake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public static void stopIntake() {
        isIntaking = false;
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public static String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double red = colors.red;
        double blue = colors.blue;
        double green = colors.green;

        if (red > blue && red > green && red > 0.01) {
            return "red";
        } else if (blue > red && blue > green && blue > 0.01) {
            return "blue";
        } else if (red > blue && green > blue && red > 0.01 && green > 0.01) {
            return "yellow";
        }
        return "";
    }

    public static void startIntakeSample() {
        isIntaking = true;
        intake();

    }

    public static void update() {
        if (isIntaking) {
            String detectedColor = getDetectedColor();
            if (detectedColor.equals(setColor) || detectedColor.equals("yellow")) {
                stopIntake();
            }
        }
    }
}

*/
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Intake {

    private static NormalizedColorSensor colorSensor;
    public static CRServo leftServo;
    public static CRServo rightServo;
    private static final String setColor = "blue";



    public static void initialize(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        leftServo = hardwareMap.get(CRServo.class, "left_servo");
        rightServo = hardwareMap.get(CRServo.class, "right_servo");
    }

    public void intake() {
        leftServo.setPower(-1);
        rightServo.setPower(1);
    }

    public void outtake() {
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }

    public void stopIntake() {
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
